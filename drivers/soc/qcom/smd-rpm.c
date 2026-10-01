// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, Sony Mobile Communications AB.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <linux/rpmsg.h>
#include <linux/soc/qcom/smd-rpm.h>

#define RPM_REQUEST_TIMEOUT     (5 * HZ)

/**
 * struct qcom_smd_rpm - state of the rpm device driver
 * @rpm_channel:	reference to the smd channel
 * @dev:		rpm device
 * @ack:		completion for acks
 * @lock:		mutual exclusion around the send/complete pair
 * @ack_status:		result of the rpm request
 */
struct qcom_smd_rpm {
	struct rpmsg_endpoint *rpm_channel;
	struct device *dev;

	struct completion ack;
	struct mutex lock;
	int ack_status;
};

/**
 * struct qcom_rpm_header - header for all rpm requests and responses
 * @service_type:	identifier of the service
 * @length:		length of the payload
 */
struct qcom_rpm_header {
	__le32 service_type;
	__le32 length;
};

/**
 * struct qcom_rpm_request - request message to the rpm
 * @msg_id:	identifier of the outgoing message
 * @flags:	active/sleep state flags
 * @type:	resource type
 * @id:		resource id
 * @data_len:	length of the payload following this header
 */
struct qcom_rpm_request {
	__le32 msg_id;
	__le32 flags;
	__le32 type;
	__le32 id;
	__le32 data_len;
};

/**
 * struct qcom_rpm_message - response message from the rpm
 * @msg_type:	indicator of the type of message
 * @length:	the size of this message, including the message header
 * @msg_id:	message id
 * @message:	textual message from the rpm
 *
 * Multiple of these messages can be stacked in an rpm message.
 */
struct qcom_rpm_message {
	__le32 msg_type;
	__le32 length;
	union {
		__le32 msg_id;
		DECLARE_FLEX_ARRAY(u8, message);
	};
};

#define RPM_SERVICE_TYPE_REQUEST	0x00716572 /* "req\0" */

#define RPM_MSG_TYPE_ERR		0x00727265 /* "err\0" */
#define RPM_MSG_TYPE_MSG_ID		0x2367736d /* "msg#" */

/**
 * qcom_rpm_smd_write - write @buf to @type:@id
 * @rpm:	rpm handle
 * @state:	active/sleep state flags
 * @type:	resource type
 * @id:		resource identifier
 * @buf:	the data to be written
 * @count:	number of bytes in @buf
 */
/*
 * NA1: RPM request trace (debug). The first na1_rpm_trace requests are logged,
 * one line per key; writing a count to /sys/module/smd_rpm/parameters/na1_rpm_trace
 * re-arms it (e.g. around a suspend). Pairs with the stock DT for the sets.
 */
static int na1_rpm_trace = 1000;
module_param(na1_rpm_trace, int, 0644);

/*
 * NA1: the last value sent per (set, type, id, key), for
 * /sys/kernel/debug/na1_rpm_state (the trace budget runs out at boot), and a
 * bisect knob, /sys/kernel/debug/na1_rpm_sleep: "TYPE ID KEY VALUE" (e.g.
 * "clka 1 swen 0") sends one SLEEP-set request; the active set is untouched.
 * Both under rpm->lock.
 */
struct na1_rpm_last {
	u32 type, id, key, val;
	u8 sleep;
	s8 ret;
};
#define NA1_RPM_LAST_MAX 256
static struct na1_rpm_last na1_rpm_last[NA1_RPM_LAST_MAX];
static unsigned int na1_rpm_nlast;
static struct qcom_smd_rpm *na1_rpm;

static void na1_rpm_record(int state, u32 type, u32 id, const void *buf,
			   size_t count, int ret)
{
	const u8 *p = buf, *end = p + count;
	u8 sleep = state == QCOM_SMD_RPM_SLEEP_STATE;
	unsigned int i;

	while (p + 8 <= end) {
		u32 key = le32_to_cpu(*(const __le32 *)p);
		u32 len = le32_to_cpu(*(const __le32 *)(p + 4));
		u32 val = (len >= 4 && p + 12 <= end) ?
			  le32_to_cpu(*(const __le32 *)(p + 8)) : 0;

		for (i = 0; i < na1_rpm_nlast; i++)
			if (na1_rpm_last[i].sleep == sleep && na1_rpm_last[i].type == type &&
			    na1_rpm_last[i].id == id && na1_rpm_last[i].key == key)
				break;
		if (i == na1_rpm_nlast && na1_rpm_nlast < NA1_RPM_LAST_MAX)
			na1_rpm_nlast++;
		if (i < na1_rpm_nlast) {
			na1_rpm_last[i] = (struct na1_rpm_last){
				.type = type, .id = id, .key = key, .val = val,
				.sleep = sleep, .ret = ret,
			};
		}
		p += 8 + ALIGN(len, 4);
	}
}

static int na1_rpm_state_show(struct seq_file *m, void *unused)
{
	unsigned int i;

	if (!na1_rpm)
		return -ENODEV;
	mutex_lock(&na1_rpm->lock);
	for (i = 0; i < na1_rpm_nlast; i++) {
		__le32 t = cpu_to_le32(na1_rpm_last[i].type);
		__le32 k = cpu_to_le32(na1_rpm_last[i].key);

		seq_printf(m, "%s %.4s %u %.4s %u ret=%d\n",
			   na1_rpm_last[i].sleep ? "sleep" : "active",
			   (const char *)&t, na1_rpm_last[i].id, (const char *)&k,
			   na1_rpm_last[i].val, na1_rpm_last[i].ret);
	}
	mutex_unlock(&na1_rpm->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(na1_rpm_state);

static u32 na1_fourcc(const char *s)
{
	u8 b[4] = {};
	int i;

	for (i = 0; i < 4 && s[i]; i++)
		b[i] = s[i];
	return b[0] | b[1] << 8 | b[2] << 16 | (u32)b[3] << 24;
}

static ssize_t na1_rpm_sleep_write(struct file *f, const char __user *ubuf,
				   size_t len, loff_t *pos)
{
	struct { __le32 key, nbytes, value; } req;
	char buf[64], type[5], key[5];
	u32 id, val;
	int ret;

	if (!na1_rpm)
		return -ENODEV;
	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	if (sscanf(buf, "%4s %u %4s %u", type, &id, key, &val) != 4)
		return -EINVAL;
	req.key = cpu_to_le32(na1_fourcc(key));
	req.nbytes = cpu_to_le32(sizeof(u32));
	req.value = cpu_to_le32(val);
	ret = qcom_rpm_smd_write(na1_rpm, QCOM_SMD_RPM_SLEEP_STATE,
				 na1_fourcc(type), id, &req, sizeof(req));
	pr_info("na1-rpm knob: sleep %s %u %s %u -> %d\n", type, id, key, val, ret);
	return ret ? ret : len;
}

static const struct file_operations na1_rpm_sleep_fops = {
	.write = na1_rpm_sleep_write,
	.llseek = noop_llseek,
};

static void na1_rpm_trace_req(int state, u32 type, u32 id, const void *buf,
			      size_t count, int ret)
{
	__le32 t = cpu_to_le32(type);
	const u8 *p = buf, *end = p + count;

	if (na1_rpm_trace <= 0)
		return;
	na1_rpm_trace--;
	while (p + 8 <= end) {
		__le32 key = *(const __le32 *)p;
		u32 len = le32_to_cpu(*(const __le32 *)(p + 4));
		u32 val = (len >= 4 && p + 12 <= end) ?
			  le32_to_cpu(*(const __le32 *)(p + 8)) : 0;

		pr_info("na1-rpm set=%s type=%.4s id=%u key=%.4s val=%u ret=%d\n",
			state == QCOM_SMD_RPM_SLEEP_STATE ? "sleep" : "active",
			(const char *)&t, id, (const char *)&key, val, ret);
		p += 8 + ALIGN(len, 4);
	}
}

int qcom_rpm_smd_write(struct qcom_smd_rpm *rpm,
		       int state,
		       u32 type, u32 id,
		       void *buf,
		       size_t count)
{
	static unsigned msg_id = 1;
	int left;
	int ret;
	struct {
		struct qcom_rpm_header hdr;
		struct qcom_rpm_request req;
		u8 payload[];
	} *pkt;
	size_t size = sizeof(*pkt) + count;

	/* SMD packets to the RPM may not exceed 256 bytes */
	if (WARN_ON(size >= 256))
		return -EINVAL;

	pkt = kmalloc(size, GFP_ATOMIC);
	if (!pkt)
		return -ENOMEM;

	mutex_lock(&rpm->lock);

	pkt->hdr.service_type = cpu_to_le32(RPM_SERVICE_TYPE_REQUEST);
	pkt->hdr.length = cpu_to_le32(sizeof(struct qcom_rpm_request) + count);

	pkt->req.msg_id = cpu_to_le32(msg_id++);
	pkt->req.flags = cpu_to_le32(state);
	pkt->req.type = cpu_to_le32(type);
	pkt->req.id = cpu_to_le32(id);
	pkt->req.data_len = cpu_to_le32(count);
	memcpy(pkt->payload, buf, count);

	ret = rpmsg_send(rpm->rpm_channel, pkt, size);
	if (ret)
		goto out;

	left = wait_for_completion_timeout(&rpm->ack, RPM_REQUEST_TIMEOUT);
	if (!left)
		ret = -ETIMEDOUT;
	else
		ret = rpm->ack_status;

out:
	kfree(pkt);
	if (rpm == na1_rpm)
		na1_rpm_record(state, type, id, buf, count, ret);
	mutex_unlock(&rpm->lock);
	na1_rpm_trace_req(state, type, id, buf, count, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_rpm_smd_write);

static int qcom_smd_rpm_callback(struct rpmsg_device *rpdev,
				 void *data,
				 int count,
				 void *priv,
				 u32 addr)
{
	const struct qcom_rpm_header *hdr = data;
	size_t hdr_length = le32_to_cpu(hdr->length);
	const struct qcom_rpm_message *msg;
	struct qcom_smd_rpm *rpm = dev_get_drvdata(&rpdev->dev);
	const u8 *buf = data + sizeof(struct qcom_rpm_header);
	const u8 *end = buf + hdr_length;
	char msgbuf[32];
	int status = 0;
	u32 len, msg_length;

	if (le32_to_cpu(hdr->service_type) != RPM_SERVICE_TYPE_REQUEST ||
	    hdr_length < sizeof(struct qcom_rpm_message)) {
		dev_err(rpm->dev, "invalid request\n");
		return 0;
	}

	while (buf < end) {
		msg = (struct qcom_rpm_message *)buf;
		msg_length = le32_to_cpu(msg->length);
		switch (le32_to_cpu(msg->msg_type)) {
		case RPM_MSG_TYPE_MSG_ID:
			break;
		case RPM_MSG_TYPE_ERR:
			len = min_t(u32, ALIGN(msg_length, 4), sizeof(msgbuf));
			memcpy_fromio(msgbuf, msg->message, len);
			msgbuf[len - 1] = 0;

			if (!strcmp(msgbuf, "resource does not exist"))
				status = -ENXIO;
			else
				status = -EINVAL;
			break;
		}

		buf = PTR_ALIGN(buf + 2 * sizeof(u32) + msg_length, 4);
	}

	rpm->ack_status = status;
	complete(&rpm->ack);
	return 0;
}

static int qcom_smd_rpm_probe(struct rpmsg_device *rpdev)
{
	struct qcom_smd_rpm *rpm;

	rpm = devm_kzalloc(&rpdev->dev, sizeof(*rpm), GFP_KERNEL);
	if (!rpm)
		return -ENOMEM;

	mutex_init(&rpm->lock);
	init_completion(&rpm->ack);

	rpm->dev = &rpdev->dev;
	rpm->rpm_channel = rpdev->ept;
	dev_set_drvdata(&rpdev->dev, rpm);

	if (!na1_rpm) {
		na1_rpm = rpm;
		debugfs_create_file("na1_rpm_state", 0400, NULL, NULL, &na1_rpm_state_fops);
		debugfs_create_file("na1_rpm_sleep", 0200, NULL, NULL, &na1_rpm_sleep_fops);
	}

	return of_platform_populate(rpdev->dev.of_node, NULL, NULL, &rpdev->dev);
}

static void qcom_smd_rpm_remove(struct rpmsg_device *rpdev)
{
	of_platform_depopulate(&rpdev->dev);
}

static const struct of_device_id qcom_smd_rpm_of_match[] = {
	{ .compatible = "qcom,glink-smd-rpm" },
	{ .compatible = "qcom,smd-rpm" },
	/*
	 * Don't add any more compatibles to the list, two previous entryes
	 * should match all defined devices.
	 */
	{ .compatible = "qcom,rpm-apq8084" },
	{ .compatible = "qcom,rpm-ipq6018" },
	{ .compatible = "qcom,rpm-ipq9574" },
	{ .compatible = "qcom,rpm-msm8226" },
	{ .compatible = "qcom,rpm-msm8909" },
	{ .compatible = "qcom,rpm-msm8916" },
	{ .compatible = "qcom,rpm-msm8936" },
	{ .compatible = "qcom,rpm-msm8953" },
	{ .compatible = "qcom,rpm-msm8974" },
	{ .compatible = "qcom,rpm-msm8976" },
	{ .compatible = "qcom,rpm-msm8994" },
	{ .compatible = "qcom,rpm-msm8996" },
	{ .compatible = "qcom,rpm-msm8998" },
	{ .compatible = "qcom,rpm-sdm660" },
	{ .compatible = "qcom,rpm-sm6115" },
	{ .compatible = "qcom,rpm-sm6125" },
	{ .compatible = "qcom,rpm-sm6375" },
	{ .compatible = "qcom,rpm-qcm2290" },
	{ .compatible = "qcom,rpm-qcs404" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smd_rpm_of_match);

static struct rpmsg_driver qcom_smd_rpm_driver = {
	.probe = qcom_smd_rpm_probe,
	.remove = qcom_smd_rpm_remove,
	.callback = qcom_smd_rpm_callback,
	.drv  = {
		.name  = "qcom_smd_rpm",
		.of_match_table = qcom_smd_rpm_of_match,
	},
};

static int __init qcom_smd_rpm_init(void)
{
	return register_rpmsg_driver(&qcom_smd_rpm_driver);
}
arch_initcall(qcom_smd_rpm_init);

static void __exit qcom_smd_rpm_exit(void)
{
	unregister_rpmsg_driver(&qcom_smd_rpm_driver);
}
module_exit(qcom_smd_rpm_exit);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm SMD backed RPM driver");
MODULE_LICENSE("GPL v2");

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for ST WLC98
 * Based on sample linux driver for ST WLC98 from ST
 */

#ifndef _STWLC98_H_
#define _STWLC98_H_

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/regmap.h>

#define WLC_DRV_VERSION			"1.0" /* driver version string format */
/* FW register address */
#define FWREG_CHIP_ID_REG		0x0000
#define FWREG_OP_MODE_ADDR		0x000E
#define FWREG_SYS_CMD_ADDR		0x0020
	/* switch_2_tx */
	#define SYS_CMD_SWITCH2TX			0x01
#define FWREG_FTP_WR_PWD_ADDR		0x0022
#define FWREG_FTP_SECTOR_INDEX_ADDR	0x0024

#define FWREG_RX_RENEGO_TARGET_PWR	0x007E
#define FWREG_RX_ERR_STATE		0x007F
/* FWREG_RX_ERR_STATE bits */
	#define ERR_STATE_OK				0x00
	#define ERR_STATE_BIDI_LAstwlc98_NO_ACK		0x10
	#define ERR_STATE_BIDI_LAstwlc98_NO_DATA	0x11
	#define ERR_STATE_BIDI_RECV_STUCK		0x12
	#define ERR_STATE_BIDI_RECV_TIMEOUT		0x13
	#define ERR_STATE_DTS_RECV_END_RST		0x20
	#define ERR_STATE_DTS_RECV_END_TIMEOUT		0x21
	#define ERR_STATE_DTS_SEND_END_RST		0x22
	#define ERR_STATE_DTS_SEND_END_TIMEOUT		0x23
#define FWREG_RX_INTR_EN		0x0080
#define FWREG_RX_INTR_CLR1		0x0084
#define FWREG_RX_INTR_CLR2		0x0085
#define FWREG_RX_INTR_CLR3		0x0086
#define FWREG_RX_INTR_CLR4		0x0087
#define FWREG_RX_INTR_LATCH1		0x0088
#define FWREG_RX_INTR_LATCH2		0x0089
#define FWREG_RX_INTR_LATCH3		0x008A
/* FWREG_RX_INTR_LATCH3 bits */
	#define RX_DTS_SEND_SUCCESS_INTR_LATCH		0x01
	#define RX_DTS_SEND_END_TIMEOUT_INTR_LATCH	0x02
	#define RX_DTS_SEND_END_RESET_INTR_LATCH	0x04
	#define RX_DTS_RCVD_SUCCESS_INTR_LATCH		0x10
	#define RX_DTS_RCVD_END_TIMEOUT_INTR_LATCH	0x20
	#define RX_DTS_RCVD_END_RESET_INTR_LATCH	0x40

#define FWREG_RX_INTR_LATCH4		0x008B
/* FWREG_RX_INTR_LATCH4 bits */
	#define RX_BIDI_RCVD_STUCK_INTR_LATCH		0x40
	#define RX_BIDI_RCVD_TIMEOUT_INTR_LATCH		0x20
	#define RX_RENEGO_AUTO_INTR_LATCH		0x10
	#define RX_RENEGO_INTR_LATCH			0x08
	#define RX_BIDI_SEND_ERR_INTR_LATCH		0x04
	#define RX_BIDI_RCVD_SUCCESS_INTR_LATCH		0x02
	#define RX_BIDI_SEND_SUCCESS_INTR_LATCH		0x01
#define FWREG_RX_STAT			0x008C
#define FWREG_RX_CMD0			0x0090
	#define RX_CMD0_VOUT_ON		0x01
	#define RX_CMD0_VOUT_OFF	0x02
	#define RX_CMD0_SEND_EPT	0x10
	#define RX_CMD0_SEND_DTS	0x20

#define FWREG_RX_CMD1			0x0091
/* FWREG_RX_CMD1 bits */
	#define RX_CMD1_SEND_BIDI			0x01
	#define RX_CMD1_RSTWLC98_BIDI			0x02
	#define RX_CMD1_RENEGO				0x08
	#define RX_CMD1_RENEGO_AUTO			0x10
#define FWREG_RX_VRECT			0x0092
#define FWREG_RX_VOUT			0x0094
#define FWREG_RX_IIN			0x0096
#define FWREG_RX_CHIP_TEMP		0x0098
#define FWREG_RX_OP_FREQ		0x009A
#define FWREG_RX_NTC			0x009C
#define FWREG_RX_DFT1			0x009E
#define FWREG_RX_DFT2			0x00A0
#define FWREG_RX_ADC_IN3		0x00A2
#define FWREG_RX_RCVD_PWR		0x00A4
#define FWREG_QI_RP_VALUE		0x00A6
#define FWREG_RX_CTRL_ERR		0x00A8
#define FWREG_QI_CE_VALUE		0x00AA
#define FWREG_RX_SIGNAL_STRENGTH	0x00AB
#define FWREG_RX_PTC_0			0x00AC
	#define PTC0_RX_GUA_PWR				0x3F
	#define PTC0_RX_GUA_PWR12W			24
#define FWREG_RX_PTC_1			0x00AD
#define FWREG_RX_PTC_2			0x00AE
#define FWREG_RX_PTC_3			0x00AF
#define FWREG_RX_PTC_4                  0x00B0
	#define PTC4_RX_NEG				0x80
#define FWREG_RX_PTC_5			0x00B1
#define FWREG_RX_PTC_6			0x00B2
#define FWREG_RX_ILIM_SET		0x00B3
#define FWREG_RX_VOUT_SET		0x00B4
	#define RX_VOUT_SET_STEP			25 /* step is 25mV */
	#define RX_VOUT_SET_LIMIT			20000
#define FWREG_RX_FOD_CUR_THRES1		0x00B6
#define FWREG_RX_FOD_CUR_THRES2		0x00B7
#define FWREG_RX_FOD_CUR_THRES3		0x00B8
#define FWREG_RX_FOD_CUR_THRES4		0x00B9
#define FWREG_RX_FOD_CUR_THRES5		0x00BA
#define FWREG_RX_FOD_GAIN0		0x00BB
#define FWREG_RX_FOD_GAIN1		0x00BC
#define FWREG_RX_FOD_GAIN2		0x00BD
#define FWREG_RX_FOD_GAIN3		0x00BE
#define FWREG_RX_FOD_GAIN4		0x00BF
#define FWREG_RX_FOD_GAIN5		0x00C0
#define FWREG_RX_FOD_OFFSET0		0x00C1
#define FWREG_RX_FOD_OFFSET1		0x00C2
#define FWREG_RX_FOD_OFFSET2		0x00C3
#define FWREG_RX_FOD_OFFSET3		0x00C4
#define FWREG_RX_FOD_OFFSET4		0x00C5
#define FWREG_RX_FOD_OFFSET5		0x00C6
#define FWREG_RX_RSER			0x00C7
#define FWREG_RX_LDO_DROP0		0x00C8
#define FWREG_RX_LDO_DROP1		0x00C9
#define FWREG_RX_LDO_DROP2		0x00CA
#define FWREG_RX_LDO_DROP3		0x00CB
#define FWREG_RX_LDO_CUR_THRES1		0x00CC
#define FWREG_RX_LDO_CUR_THRES2		0x00CD
#define FWREG_RX_LDO_CUR_THRES3		0x00CE
#define FWREG_RX_EPT_MSG		0x00CF
	#define EPT_MSG_GOOGLE				0xF3
#define FWREG_RX_GOOGLE_FEATURE		0x00D7
	#define  GOOGLE_FEATURE_RP0			0x01
#define FWREG_RX_DTS_SEND		0x00D8
#define FWREG_RX_DTS_SEND_REQUEST	0x00DA
#define FWREG_RX_DTS_RCVD		0x00DC
#define FWREG_RX_DTS_RCVD_REQUEST	0x00DE
#define FWREG_ARC_AUTO_OFF_THRES	0x00E0

#define FWREG_RX_BIDI_SEND_LEN		0x00F6
#define FWREG_RX_BIDI_SEND_CNT		0x00F7
#define FWREG_RX_BIDI_RCVD_LEN		0x00F8
#define FWREG_RX_BIDI_RCVD_CNT		0x00F9
#define FWREG_RX_GGL_COMM_STAT		0x00FA
	#define COMM_STAT_IDLE				0x00
	#define COMM_STAT_BIDI_SEND_START		0x10
	#define COMM_STAT_BIDI_SEND_TRANSFER		0x11
	#define COMM_STAT_BIDI_SEND_ERRORACK		0x12
	#define COMM_STAT_BIDI_SEND_RESET		0x13
	#define COMM_STAT_BIDI_RCVD_LISTEN		0x20
	#define COMM_STAT_BIDI_RCVD_START		0x21
	#define COMM_STAT_BIDI_RCVD_TRANSFER		0x22
	#define COMM_STAT_BIDI_RCVD_END			0x23
	/* 0x24: "BiDi Receive Error",
	 * 0x25: "BiDi Receive Reset",
	 * 0x30: "DTS Send",
	 * 0x40: "DTS Receive Listen",
	 * 0x41: "DTS Receive Transfer",
	 * 0x50: "Re-Nego",
	 * 0x60: "Proprietary Packet",
	 */
/* tx */
#define FWREG_TX_INTR_EN		0x0100
	#define TX_INTR_TXEN				0x01
	#define TX_INTR_TXDIS				0x02
#define FWREG_TX_INTR_CLR		0x0104
#define FWREG_TX_INTR_LATCH		0x0108
#define FWREG_TX_STAT			0x010C
#define FWREG_TX_CMD			0x0110
#define FWREG_TX_EPT_REASON_RCVD1	0x0112
#define FWREG_TX_EPT_REASON_RCVD2	0x0113
#define FWREG_TX_EPT_REASON_RCVD3	0x0114
#define FWREG_TX_RECENT_CEP		0x0115
#define FWREG_TX_VRECT			0x0116
#define FWREG_TX_VIN			0x0118
#define FWREG_TX_IOUT			0x011A
#define FWREG_TX_CHIP_TEMP		0x011C
#define FWREG_TX_OP_FREQ		0x011E
#define FWREG_TX_NTC			0x0120
#define FWREG_TX_DFT1			0x0122
#define FWREG_TX_DFT2			0x0124
#define FWREG_TX_ADC_IN3		0x0126
#define FWREG_TX_PWR_TFRD_TO_RX		0x0128
#define FWREG_TX_PWR_RCVD_BY_RX		0x012A
#define FWREG_TX_OP_DC			0x012C
#define FWREG_TX_PTC_0			0x012D
#define FWREG_TX_PTC_1			0x012E
#define FWREG_TX_PTC_2			0x012F
#define FWREG_TX_PTC_3			0x0130
#define FWREG_TX_PTC_4			0x0131
#define FWREG_TX_PTC_5			0x0132
#define FWREG_TX_PTC_6			0x0133
#define FWREG_TX_CTRL			0x0134
#define FWREG_TX_BRIDGE_MODE_CUR_THRES	0x0135
#define FWREG_TX_BRIDGE_STAT		0x0136
#define FWREG_TX_OVP_THRES		0x0140
#define FWREG_TX_OCP_THRES		0x0141
#define FWREG_TX_OVTP_THRES		0x0142
#define FWREG_TX_PID_MAX_CUR		0x0143
#define FWREG_TX_MAX_FREQ		0x0144
#define FWREG_TX_MIN_FREQ		0x0145
#define FWREG_TX_PING_FREQ		0x0146
#define FWREG_TX_PING_INTERVAL		0x0147
#define FWREG_TX_MAX_DC			0x0148
#define FWREG_TX_MIN_DC			0x0149
#define FWREG_TX_FOD_PLOSS_THRES	0x014A
#define FWREG_TX_FOD_DBNC_CNT		0x014B
#define FWREG_TX_CE_TO_MAX_CNT		0x014C
#define FWREG_TX_FHOP_STEP		0x014D
#define FWREG_TX_PING_DC		0x014E
#define FWREG_TX_DTS_SEND		0x0150
#define FWREG_TX_DTS_RCVD		0x0154
#define FWREG_Q_EXCITE_FREQ		0x015A
#define FWREG_Q_MEAS_RESONANT_FREQ	0x015C
#define FWREG_Q_MEAS_QFACTOR		0x015E
#define FWREG_Q_MEAS_ADC		0x0160

#define FWREG_AUX_DATA_00		0x0180
#define FWREG_DTS_SEND_DATA_00		0x0200

#define FWREG_BIDI_SEND_DATA_00		0x0300
#define FWREG_BIDI_RECV_DATA_00		0x0380
#define FWREG_MAX_REGISTER		0x03FF

/* error code */
/* bidi */
#define E_BIDI_SEND_ERR			(0x00000001)
#define E_BIDI_SEND_TIMEOUT		(0x00000002)
#define E_BIDI_RCVD_TIMEOUT		(0x00000003)
#define E_BIDI_RCVD_STUCK		(0x00000004)
#define E_RENEG_TIMEOUT			(0x00000005)
#define E_DTS_SEND_END_TIMEOUT		(0x00000006)
#define E_DTS_SEND_END_RESET		(0x00000007)
#define E_DTS_RCVD_END_TIMEOUT		(0x00000008)
#define E_DTS_RCVD_END_RESET		(0x00000009)
#define E_VOUT_SET			(0x0000000A)
#define E_BUS_R				(0x80000001)
#define E_BUS_W				(0x80000002)
#define E_BUS_WR			(0x80000003)
#define E_UNEXPECTED_OP_MODE		(0x80000004)
#define E_FTP_WRITE			(0x80000005)
#define E_INVALID_INPUT			(0x80000006)
#define E_MEMORY_ALLOC			(0x80000007)
#define E_UNEXPECTED_HW_REV		(0x80000008)
#define E_TIMEOUT			(0x80000009)
#define E_FTP_DATA_MISMATCH		(0x8000000A)
#define E_FTP_DATA_CORRUPTION		(0x8000000B)
#define E_FTP_ERASE			(0x8000000C)
/* timeout 30sec */
#define DELAY_CNT			30
#define DELAY_EACH_TIME			1000


/* SYSREG registers */
#define OPCODE_WRITE			0xFA
#define HWREG_HW_VER_ADDR		0x2001C000
#define HWREG_PMU_REG0			0x2001C180
	#define PMU_REG0_DISABLE_EXTLDO				0x20
	#define PMU_REG0_ENTER					0x30
	#define PMU_REG0_ENABLE_EXTLDO				0x32

#define MAX_CMD_SIZE			200

struct wlc_chip_info {
	u16 chip_id;
	u8 chip_revision;
	u8 customer_id;
	u16 project_id;
	u16 ftp_patch_id;
	u16 ram_patch_id;
	u16 config_id;
	u16 pe_id;
	u8 cut_id;
};

struct wlc_ts_info {
	struct device *dev;
	struct i2c_client *client;
	struct wlc_chip_info chip_info;
	struct attribute_group attrs; /* SysFS attributes */
	int irq_gpio; /* number of the gpio associated to the interrupt pin */
	int irq;
	u8 rx_intr_latch_value_0x88;
	u8 rx_intr_latch_value_0x89;
	u8 rx_intr_latch_value_0x8A; /* auth */
	u8 rx_intr_latch_value_0x8B; /* bidi and re-neg */
	struct regmap *regmap;

	/* debug */
	struct dentry *debug_root;
	u32 debug_address;
};

#endif /* _STWLC98_H_ */


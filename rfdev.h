#ifndef RFDEV_H
#define RFDEV_H

/* Each PIC operation corresponds to a dummy client */
enum i2c_client_write_opr {
        PIC_WR_BUF_DATA = 0,
        PIC_WR_DEFAULT,
        PIC_WR_TMS_OUT,
        PIC_WR_TMS_OUT_LEN,
        PIC_WR_TDI_TDO,
        PIC_WR_TDI_TDO_LEN,
        PIC_WR_TDI_OUT,
        PIC_WR_TDI_OUT_LEN,
        PIC_WR_TDI_TDO_CONT,
        PIC_WR_TDI_TDO_LEN_CONT,
        PIC_WR_TDI_OUT_CONT,
        PIC_WR_TDI_OUT_LEN_CONT,
        PIC_WR_FEAT_UPDATE = 16,
};

enum i2c_client_read_opr {
        PIC_RD_BUF_DATA = 0,
        PIC_RD_V = 5,
        PIC_RD_TDO_IN,
        PIC_RD_TDO_IN_LEN,
        PIC_RD_TDO_IN_CONT = 10,
        PIC_RD_TDO_IN_LEN_CONT,
        PIC_RD_F = 16,
};

#endif

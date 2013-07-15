/* Brody's note - This file following this comment is taken directly from the Freescale website.
 * @ http://cache.freescale.com/files/sensors/software_tools/MMA7660FC_RDHF.h?fr=gtl
 * With one exception -- the "PD 0x0A" section which clashes with Atmel/Arduino PINs of the same name are modified.
 * This code is not (necessarily?) in the public domain.
 * */
/* Based on MMA7660FC */
/*
** ###################################################################
**     Filename  : MMA7660.h
**     DataSheet : MMA7660FC Rev 3 Release Date: April 2009
**     Compiler  : CodeWarrior compiler
**     Date/Time : 5/14/09, 11:11 AM
**     Abstract  :
**         This header implements the register set for the MMA7660FC.
**
** ###################################################################
*/

#ifndef _MMA7660_H
#define _MMA7660_H

/****************************************Registers****************************************/
#define XOUT        0x00
#define YOUT        0x01
#define ZOUT        0x02
#define TILT        0x03
#define SRST        0x04
#define SPCNT       0x05
#define INTSU       0x06
#define MODE        0x07
#define SR          0x08
#define PDET        0x09
#define PD          0x0A


/********************XOUT 0x00********************/
typedef union {
 byte Byte;
 struct{
   byte XOUT0           :1;
   byte XOUT1           :1;
   byte XOUT2           :1;
   byte XOUT3           :1;
   byte XOUT4           :1;
   byte XOUT5           :1;
   byte ALERT           :1;
   byte                 :1;
 }  Bits;
 struct {
   byte  grpXOUT        :6;
   byte                 :1;
   byte                 :1;
 }  Merged;
} XOUT_STR;

extern volatile XOUT_STR _XOUT;
#define XOUT_REG               _XOUT.Byte
#define XOUT_XOUT0          _XOUT.bits.XOUT0
#define XOUT_XOUT1          _XOUT.bits.XOUT1
#define XOUT_XOUT2          _XOUT.bits.XOUT2
#define XOUT_XOUT3          _XOUT.bits.XOUT3
#define XOUT_XOUT4          _XOUT.bits.XOUT4
#define XOUT_XOUT5          _XOUT.bits.XOUT5
#define XOUT_ALERT          _XOUT.bits.ALERT

#define XOUT_XOUT           _XOUT.Merged.grpXOUT

#define XOUT_XOUT0_MASK             1
#define XOUT_XOUT1_MASK             2
#define XOUT_XOUT2_MASK             4
#define XOUT_XOUT3_MASK             8
#define XOUT_XOUT4_MASK             16
#define XOUT_XOUT5_MASK             32
#define XOUT_ALERT_MASK             64

#define XOUT_XOUT_MASK      127


/********************YOUT 0x01********************/
typedef union {
 byte Byte;
 struct{
   byte YOUT0           :1;
   byte YOUT1           :1;
   byte YOUT2           :1;
   byte YOUT3           :1;
   byte YOUT4           :1;
   byte YOUT5           :1;
   byte ALERT           :1;
   byte                 :1;
 }  Bits;
 struct {
   byte  grpYOUT        :6;
   byte                 :1;
   byte                 :1;
 }  Merged;
} YOUT_STR;

extern volatile YOUT_STR _YOUT;
#define YOUT_REG               _YOUT.Byte
#define YOUT_YOUT0          _YOUT.bits.YOUT0
#define YOUT_YOUT1          _YOUT.bits.YOUT1
#define YOUT_YOUT2          _YOUT.bits.YOUT2
#define YOUT_YOUT3          _YOUT.bits.YOUT3
#define YOUT_YOUT4          _YOUT.bits.YOUT4
#define YOUT_YOUT5          _YOUT.bits.YOUT5
#define YOUT_ALERT          _YOUT.bits.ALERT

#define YOUT_YOUT           _YOUT.Merged.grpYOUT

#define YOUT_YOUT0_MASK             1
#define YOUT_YOUT1_MASK             2
#define YOUT_YOUT2_MASK             4
#define YOUT_YOUT3_MASK             8
#define YOUT_YOUT4_MASK             16
#define YOUT_YOUT5_MASK             32
#define YOUT_ALERT_MASK             64


#define YOUT_YOUT_MASK      127


/********************ZOUT 0x02********************/
typedef union {
 byte Byte;
 struct{
   byte ZOUT0           :1;
   byte ZOUT1           :1;
   byte ZOUT2           :1;
   byte ZOUT3           :1;
   byte ZOUT4           :1;
   byte ZOUT5           :1;
   byte ALERT           :1;
   byte                 :1;
 }  Bits;
 struct {
   byte  grpZOUT        :6;
   byte                 :1;
   byte                 :1;
 }  Merged;
} ZOUT_STR;

extern volatile ZOUT_STR _ZOUT;
#define ZOUT_REG            _ZOUT.Byte
#define ZOUT_ZOUT0          _ZOUT.bits.ZOUT0
#define ZOUT_ZOUT1          _ZOUT.bits.ZOUT1
#define ZOUT_ZOUT2          _ZOUT.bits.ZOUT2
#define ZOUT_ZOUT3          _ZOUT.bits.ZOUT3
#define ZOUT_ZOUT4          _ZOUT.bits.ZOUT4
#define ZOUT_ZOUT5          _ZOUT.bits.ZOUT5
#define ZOUT_ALERT          _ZOUT.bits.ALERT

#define ZOUT_ZOUT           _ZOUT.Merged.grpZOUT

#define ZOUT_ZOUT0_MASK             1
#define ZOUT_ZOUT1_MASK             2
#define ZOUT_ZOUT2_MASK             4
#define ZOUT_ZOUT3_MASK             8
#define ZOUT_ZOUT4_MASK             16
#define ZOUT_ZOUT5_MASK             32
#define ZOUT_ALERT_MASK             64


#define ZOUT_ZOUT_MASK      127


/********************TILT 0x03********************/
typedef union {
 byte Byte;
 struct{
   byte BAFRO0           :1;
   byte BAFRO1           :1;
   byte POLA0            :1;
   byte POLA1            :1;
   byte POLA2            :1;
   byte PULSE            :1;
   byte ALERT            :1;
   byte SHAKE            :1;
 }  Bits;
 struct {
   byte  grpBAFRO        :2;
   byte  grpPOLA         :3;
   byte                  :3;
 }  Merged;
} TILT_STR;

extern volatile TILT_STR _TILT;
#define TILT_REG            _TILT.Byte
#define TILT_BAFRO0         _TILT.bits.BAFRO0
#define TILT_BAFRO1         _TILT.bits.BAFRO1
#define TILT_POLA0          _TILT.bits.POLA0
#define TILT_POLA1          _TILT.bits.POLA1
#define TILT_POLA2          _TILT.bits.POLA2
#define TILT_PULSE          _TILT.bits.PULSE
#define TILT_ALERT          _TILT.bits.ALERT
#define TILT_SHAKE          _TILT.bits.SHAKE

#define TILT_BAFRO          _TILT.Merged.grpBAFRO
#define TILT_POLA           _TILT.Merged.grpPOLA

#define TILT_BAFRO0_MASK            1
#define TILT_BAFRO1_MASK            2
#define TILT_POLA0_MASK             4
#define TILT_POLA1_MASK             8
#define TILT_POLA2_MASK             16
#define TILT_PULSE_MASK             32
#define TILT_ALERT_MASK             64
#define TILT_SHAKE_MASK             128

#define TILT_BAFRO_MASK              3
#define TILT_POLA_MASK              28


/********************SRST 0x04********************/
typedef union {
 byte Byte;
 struct{
   byte AMSRS           :1;
   byte AWSRS           :1;
   byte                 :1;
   byte                 :1;
   byte                 :1;
   byte                 :1;
   byte                 :1;
   byte                 :1;
 }  Bits;
 struct {
   byte                 :8;
 }  Merged;
} SRST_STR;

extern volatile SRST_STR _SRST;
#define SRST_REG           _SRST.Byte
#define SRST_AMSRS         _SRST.bits.AMSRS
#define SRST_AWSRS         _SRST.bits.AWSRS

#define SRST_AMSRS_MASK            1
#define SRST_AWSRS_MASK            2


/********************SPCNT 0x05********************/

typedef union {
 byte Byte;
 struct{
   byte SC0            :1;
   byte SC1            :1;
   byte SC2            :1;
   byte SC3            :1;
   byte SC4            :1;
   byte SC5            :1;
   byte SC6            :1;
   byte SC7            :1;
 }  Bits;
 struct {
   byte  grpSC         :8;
 }  Merged;
} SPCNT_STR;

extern volatile SPCNT_STR _SPCNT;
#define SPCNT_REG         _SPCNT.Byte
#define SPCNT_SC0         _SPCNT.bits.SC0
#define SPCNT_SC1         _SPCNT.bits.SC1
#define SPCNT_SC2         _SPCNT.bits.SC2
#define SPCNT_SC3         _SPCNT.bits.SC3
#define SPCNT_SC4         _SPCNT.bits.SC4
#define SPCNT_SC5         _SPCNT.bits.SC5
#define SPCNT_SC6         _SPCNT.bits.SC6
#define SPCNT_SC7         _SPCNT.bits.SC7

#define SPCNT_SC          _SPCNT.Merged.grpSC

#define SPCNT_SC0_MASK             1
#define SPCNT_SC1_MASK             2
#define SPCNT_SC2_MASK             4
#define SPCNT_SC3_MASK             8
#define SPCNT_SC4_MASK             16
#define SPCNT_SC5_MASK             32
#define SPCNT_SC6_MASK             64
#define SPCNT_SC7_MASK             128

#define SPCNT_SC_MASK              255



/********************INTSU 0x06********************/

typedef union {
 byte Byte;
 struct{
   byte FBINT            :1;
   byte PLINT            :1;
   byte PDINT            :1;
   byte ASINT            :1;
   byte GINT             :1;
   byte SHINTZ           :1;
   byte SHINTY           :1;
   byte SHINTX           :1;
 }  Bits;
 struct {
   byte                  :8;
 }  Merged;
} INTSU_STR;

extern volatile INTSU_STR _INTSU;
#define INTSU_REG           _INTSU.Byte
#define INTSU_FBINT         _INTSU.bits.FBINT
#define INTSU_PLINT         _INTSU.bits.PLINT
#define INTSU_PDINT         _INTSU.bits.PDINT
#define INTSU_ASINT         _INTSU.bits.ASINT
#define INTSU_GINT          _INTSU.bits.GINT
#define INTSU_SHINTZ        _INTSU.bits.SHINTZ
#define INTSU_SHINTY        _INTSU.bits.SHINTY
#define INTSU_SHINTX        _INTSU.bits.SHINTX


#define INTSU_FBINT_MASK            1
#define INTSU_PLINT_MASK            2
#define INTSU_PDINT_MASK            4
#define INTSU_ASINT_MASK            8
#define INTSU_GINT_MASK             16
#define INTSU_SHINTZ_MASK           32
#define INTSU_SHINTY_MASK           64
#define INTSU_SHINTX_MASK           128


/********************MODE 0x07********************/

typedef union {
 byte Byte;
 struct{
   byte MDE            :1;
   byte                :1;
   byte TON            :1;
   byte AWE            :1;
   byte ASE            :1;
   byte SCPS           :1;
   byte IPP            :1;
   byte IAH            :1;
 }  Bits;
 struct {
   byte                  :8;
 }  Merged;
} MODE_STR;

extern volatile MODE_STR _MODE;
#define MODE_REG           _MODE.Byte
#define MODE_MODE        _MODE.bits.MDE
#define MODE_TON         _MODE.bits.TON
#define MODE_AWE         _MODE.bits.AWE
#define MODE_ASE         _MODE.bits.ASE
#define MODE_SCPS        _MODE.bits.SCPS
#define MODE_IPP         _MODE.bits.IPP
#define MODE_IAH         _MODE.bits.IAH


#define MODE_MODE_MASK           1
#define MODE_TON_MASK            4
#define MODE_AWE_MASK            8
#define MODE_ASE_MASK            16
#define MODE_SCPS_MASK           32
#define MODE_IPP_MASK            64
#define MODE_IAH_MASK            128


/********************SR 0x08********************/
typedef union {
 byte Byte;
 struct{
   byte AMSR0            :1;
   byte AMSR1            :1;
   byte AMSR2            :1;
   byte AWSR0            :1;
   byte AWSR1            :1;
   byte FILT0            :1;
   byte FILT1            :1;
   byte FILT2            :1;
 }  Bits;
 struct {
   byte  grpAMSR         :3;
   byte  grpAWSR         :2;
   byte  grpFILT         :3;
 }  Merged;
} SR_STR;

extern volatile SR_STR _SR;
#define SR_REG           _SR.Byte
#define SR_AMSR0         _SR.bits.AMSR0
#define SR_AMSR1         _SR.bits.AMSR1
#define SR_AMSR2         _SR.bits.AMSR2
#define SR_AWSR0         _SR.bits.AWSR0
#define SR_AWSR1         _SR.bits.AWSR1
#define SR_FILT0         _SR.bits.FILT0
#define SR_FILT1         _SR.bits.FILT1
#define SR_FILT2         _SR.bits.FILT2

#define SR_AMSR          _SR.Merged.grpAMSR
#define SR_AWSR          _SR.Merged.grpAWSR
#define SR_FILT          _SR.Merged.grpFILT

#define SR_AMSR0_MASK             1
#define SR_AMSR1_MASK             2
#define SR_AMSR2_MASK             4
#define SR_AWSR0_MASK             8
#define SR_AWSR1_MASK             16
#define SR_FILT0_MASK             32
#define SR_FILT1_MASK             64
#define SR_FILT2_MASK             128

#define SR_AMSR_MASK              7
#define SR_AWSR_MASK              24
#define SR_FILT_MASK              224



/********************PDET 0x09********************/
typedef union {
 byte Byte;
 struct{
   byte PDTH0            :1;
   byte PDTH1            :1;
   byte PDTH2            :1;
   byte PDTH3            :1;
   byte PDTH4            :1;
   byte XDA              :1;
   byte YDA              :1;
   byte ZDA              :1;
 }  Bits;
 struct {
   byte  grpPDTH         :5;
   byte                  :3;
 }  Merged;
} PDET_STR;

extern volatile PDET_STR _PDET;
#define PDET_REG           _PDET.Byte
#define PDET_PDTH0         _PDET.bits.PDTH0
#define PDET_PDTH1         _PDET.bits.PDTH1
#define PDET_PDTH2         _PDET.bits.PDTH2
#define PDET_PDTH3         _PDET.bits.PDTH3
#define PDET_PDTH4         _PDET.bits.PDTH4
#define PDET_XDA           _PDET.bits.XDA
#define PDET_YDA           _PDET.bits.YDA
#define PDET_ZDA           _PDET.bits.ZDA

#define PDET_PDTH          _PDET.Merged.grpPDTH

#define PDET_PDTH0_MASK             1
#define PDET_PDTH1_MASK             2
#define PDET_PDTH2_MASK             4
#define PDET_PDTH3_MASK             8
#define PDET_PDTH4_MASK             16
#define PDET_XDA_MASK               32
#define PDET_YDA_MASK               64
#define PDET_ZDA_MASK               128

#define PDET_AMPDET_MASK            31


/********************PD 0x0A********************/
typedef union {
 byte Byte;
 struct{
#if 0 /*These clash with Arduino pins*/
   byte PD0            :1;
   byte PD1            :1;
   byte PD2            :1;
   byte PD3            :1;
   byte PD4            :1;
   byte PD5            :1;
   byte PD6            :1;
   byte PD7            :1;
#else
   byte PD0bk            :1;
   byte PD1bk            :1;
   byte PD2bk            :1;
   byte PD3bk            :1;
   byte PD4bk            :1;
   byte PD5bk            :1;
   byte PD6bk            :1;
   byte PD7bk            :1;
#endif
 }  Bits;
 struct {
   byte  grpPD         :8;
 }  Merged;
} PD_STR;

extern volatile PD_STR _PD;
#define PD_REG         _PD.Byte
#if 0 /*BK: See above.*/
#define PD_PD0         _PD.bits.PD0
#define PD_PD1         _PD.bits.PD1
#define PD_PD2         _PD.bits.PD2
#define PD_PD3         _PD.bits.PD3
#define PD_PD4         _PD.bits.PD4
#define PD_PD5         _PD.bits.PD5
#define PD_PD6         _PD.bits.PD6
#define PD_PD7         _PD.bits.PD7
#else
#define PD_PD0         _PD.bits.PD0bk
#define PD_PD1         _PD.bits.PD1bk
#define PD_PD2         _PD.bits.PD2bk
#define PD_PD3         _PD.bits.PD3bk
#define PD_PD4         _PD.bits.PD4bk
#define PD_PD5         _PD.bits.PD5bk
#define PD_PD6         _PD.bits.PD6bk
#define PD_PD7         _PD.bits.PD7bk
#endif

#define PD_PD          _PD.Merged.grpPD

#define PD_PD0_MASK             1
#define PD_PD1_MASK             2
#define PD_PD2_MASK             4
#define PD_PD3_MASK             8
#define PD_PD4_MASK             16
#define PD_PD5_MASK             32
#define PD_PD6_MASK             64
#define PD_PD7_MASK             128

#define PD_PD_MASK              255




#endif    //_MMA7660_H

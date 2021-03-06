# Hey Emacs, this is a -*- makefile -*-

# XSens Mti-G

#    <subsystem name="ins" type="xsens">
#      <configure name="XSENS_UART_NR" value="0"/>
#      <configure name="XSENS_UART_BAUD" value="B115200"/>
#    </subsystem>



#########################################
## ATTITUDE

ap.CFLAGS += -DUSE_INS_MODULE

# AHRS Results
ap.CFLAGS += -DINS_TYPE_H=\"modules/ins/ins_xsens.h\"

ifndef XSENS_UART_BAUD
	XSENS_UART_BAUD = B115200
endif

#B230400
#B115200

ap.CFLAGS += -DUSE_UART$(XSENS_UART_NR)
ap.CFLAGS += -DINS_LINK=Uart$(XSENS_UART_NR)
ap.CFLAGS += -DUART$(XSENS_UART_NR)_BAUD=$(XSENS_UART_BAUD)
ap.CFLAGS += -DXSENS_OUTPUT_MODE=0x1836
ap.srcs   += $(SRC_MODULES)/ins/ins_xsens.c
ap.CFLAGS += -DAHRS_TRIGGERED_ATTITUDE_LOOP



ifeq ($(TARGET), fbw)

# when compiling FBW only, the settings need to know the AHRS_TYPE

fbw.CFLAGS += -DAHRS_TYPE_H=\"modules/ins/ins_xsens.h\"

endif


#########################################
## GPS

ap.CFLAGS += -DUSE_GPS_XSENS
ap.CFLAGS += -DUSE_GPS_XSENS_RAW_DATA
ap.CFLAGS += -DGPS_NB_CHANNELS=16
ap.CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
ap.CFLAGS += -DGPS_TYPE_H=\"modules/ins/ins_xsens.h\"
ap.srcs += $(SRC_SUBSYSTEMS)/gps.c


#########################################
## Simulator

sim.CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_sim.h\"
sim.CFLAGS += -DUSE_AHRS -DAHRS_UPDATE_FW_ESTIMATOR

sim.srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
sim.srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_sim.c

sim.srcs   += $(SRC_SUBSYSTEMS)/ins/ins_gps_passthrough.c

sim.CFLAGS += -DUSE_GPS -DGPS_USE_LATLONG
sim.CFLAGS += -DGPS_TYPE_H=\"subsystems/gps/gps_sim.h\"
sim.srcs += $(SRC_SUBSYSTEMS)/gps/gps_sim.c
sim.srcs += $(SRC_SUBSYSTEMS)/gps.c






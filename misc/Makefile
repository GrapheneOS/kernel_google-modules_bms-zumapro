# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Google Battery Management System
#
GBMS_TOOLS_MODULES =	GOOGLE_VOTABLE \
			GOOGLE_LOGBUFFER \


# gvotable
obj-$(CONFIG_GOOGLE_VOTABLE) += gvotable.o

# logbuffer
obj-$(CONFIG_GOOGLE_LOGBUFFER)	+= logbuffer.o


CFLAGS_gvotable.o += $(WENUMS)
CFLAGS_logbuffer.o += $(WENUMS)

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += $(foreach m,$(GBMS_TOOLS_MODULES),CONFIG_$(m)=m )

EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS += $(foreach m,$(GBMS_TOOLS_MODULES),-DCONFIG_$(m)_MODULE)

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" $(@)

print-%:
	@echo $* = $($*)

value-%:
	@echo $($*)

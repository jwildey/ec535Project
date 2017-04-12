export EC535PROJ_ROOT = .

include $(EC535PROJ_ROOT)/mk/config.mk

all clean: $(EC535PROJ_DIRS)
	$(MAKE) -C src $(@)

$(EC535PROJ_DIRS):
	mkdir -p $(EC535PROJ_DIRS)

clobber:
	$(RM) -r obj bin

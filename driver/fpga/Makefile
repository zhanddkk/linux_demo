include project.mk

export KNLNEL_ROOT CROSS_COMPILE ARCH PROJ_PATH COMPILE_TIME INSTALL_MOD_DIR INSTALL_MOD_PATH BUILD_PHONY

SUB_SRC = jpeg peak
SUB_INSTALL = $(addprefix install-, $(SUB_SRC))
SUB_CLEAN = $(addprefix clean-, $(SUB_SRC))
BUILD_PHONY += all install clean $(SUB_SRC) $(SUB_INSTALL) $(SUB_CLEAN)

all: $(SUB_SRC)
	@echo "build all finished"

install: $(SUB_INSTALL)
	@echo "install finished"

clean: $(SUB_CLEAN)
	@echo "clean all finished"


	
$(SUB_SRC):
	${MAKE}	-C $@ all

$(SUB_INSTALL):
	${MAKE} -C $(patsubst install-%,%,$@) install

$(SUB_CLEAN):
	${MAKE} -C $(patsubst clean-%,%,$@) clean

.PHONY: ${BUILD_PHONY}

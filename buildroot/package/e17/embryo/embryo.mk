################################################################################
#
# embryo -- EFL EMBRYO library
#
################################################################################

EFL_EMBRYO_VERSION = 1.0.0
EFL_EMBRYO_SOURCE = embryo-$(EFL_EMBRYO_VERSION).tar.gz
EFL_EMBRYO_SITE = http://download.enlightenment.org/releases/
EFL_EMBRYO_AUTORECONF = NO
EFL_EMBRYO_INSTALL_STAGING = YES
EFL_EMBRYO_INSTALL_TARGET = YES
EFL_EMBRYO_DEPENDENCIES = pkg-config

$(eval $(call AUTOTARGETS,package/efl,efl_embryo))
$(eval $(call AUTOTARGETS,package/efl,efl_embryo,host))

################################################################################
#
# efreet -- EFL EFREET library
#
################################################################################

EFL_EFREET_VERSION = 1.0.0
EFL_EFREET_SOURCE = efreet-$(EFL_EFREET_VERSION).tar.gz
EFL_EFREET_SITE = http://download.enlightenment.org/releases/
EFL_EFREET_AUTORECONF = NO
EFL_EFREET_INSTALL_STAGING = YES
EFL_EFREET_INSTALL_TARGET = YES
EFL_EFREET_DEPENDENCIES = pkg-config efl_eina efl_eet efl_ecore 

ifeq ($(BR2_PACKAGE_EFL_EFREET_COVERAGE),y)
EFL_EFREET_CONF_OPT += --enable-coverage
endif

ifeq ($(BR2_PACKAGE_EFL_EFREET_TESTS),y)
EFL_EFREET_CONF_OPT += --enable-tests
endif


$(eval $(call AUTOTARGETS,package/efl,efl_efreet))
$(eval $(call AUTOTARGETS,package/efl,efl_efreet,host))

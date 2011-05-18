################################################################################
#
# eina -- EFL EET library
#
################################################################################

EFL_EET_VERSION = 1.4.0
EFL_EET_SOURCE = eet-$(EFL_EET_VERSION).tar.gz
EFL_EET_SITE = http://download.enlightenment.org/releases/
EFL_EET_AUTORECONF = NO
EFL_EET_INSTALL_STAGING = YES
EFL_EET_INSTALL_TARGET = YES
EFL_EET_DEPENDENCIES = pkg-config jpeg openssl

ifeq ($(BR2_PACKAGE_EFL_EET_EXAMPLES),y)
EFL_EET_CONF_OPT += --enable-build-examples
endif

ifeq ($(BR2_PACKAGE_EFL_EET_COVERAGE),y)
EFL_EET_CONF_OPT += --enable-coverage
endif

ifeq ($(BR2_PACKAGE_EFL_EET_TESTS),y)
EFL_EET_CONF_OPT += --enable-tests
endif


$(eval $(call AUTOTARGETS,package/efl,efl_eet))
$(eval $(call AUTOTARGETS,package/efl,efl_eet,host))

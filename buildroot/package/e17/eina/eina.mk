################################################################################
#
# eina -- EFL EINA library
#
################################################################################

EFL_EINA_VERSION = 1.0.0
EFL_EINA_SOURCE = eina-$(EFL_EINA_VERSION).tar.gz
EFL_EINA_SITE = http://download.enlightenment.org/releases/
EFL_EINA_AUTORECONF = NO
EFL_EINA_INSTALL_STAGING = YES
EFL_EINA_INSTALL_TARGET = YES
EFL_EINA_DEPENDENCIES = pkg-config

ifeq ($(BR2_PACKAGE_EFL_EINA_TESTS),y)
EFL_EINA_CONF_OPT += --enable-tests
endif

ifeq ($(BR2_PACKAGE_EFL_EINA_COVERAGE),y)
EFL_EINA_CONF_OPT += --enable-coverage
endif

ifeq ($(BR2_PACKAGE_EFL_EINA_BENCHMARK),y)
EFL_EINA_CONF_OPT += --enable-benchmark
endif


$(eval $(call AUTOTARGETS,package/efl,efl_eina))
$(eval $(call AUTOTARGETS,package/efl,efl_eina,host))

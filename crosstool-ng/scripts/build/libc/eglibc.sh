# eglibc build functions (initially by Thomas JOURDAN).

# Download eglibc repository
do_eglibc_get() {
    CT_HasOrAbort svn

    case "${CT_LIBC_VERSION}" in
        trunk)  svn_url="svn://svn.eglibc.org/trunk";;
        *)      svn_url="svn://svn.eglibc.org/branches/eglibc-${CT_LIBC_VERSION}";;
    esac

    case "${CT_EGLIBC_CHECKOUT}" in
        y)  svn_action="checkout";;
        *)  svn_action="export --force";;
    esac

    CT_DoExecLog ALL svn ${svn_action} -r "${CT_EGLIBC_REVISION:-HEAD}" "${svn_url}" "$(pwd)"

    # Compress eglibc
    CT_DoExecLog ALL mv libc "eglibc-${CT_LIBC_VERSION}"
    CT_DoExecLog ALL tar cjf "eglibc-${CT_LIBC_VERSION}.tar.bz2" "eglibc-${CT_LIBC_VERSION}"

    # Compress linuxthreads, localedef and ports
    # Assign them the name the way ct-ng like it
    for addon in linuxthreads localedef ports; do
        CT_DoExecLog ALL mv "${addon}" "eglibc-${addon}-${CT_LIBC_VERSION}"
        CT_DoExecLog ALL tar cjf "eglibc-${addon}-${CT_LIBC_VERSION}.tar.bz2" "eglibc-${addon}-${CT_LIBC_VERSION}"
    done
}

# Download glibc
do_libc_get() {
    # eglibc is only available through subversion, there are no
    # snapshots available. Moreover, addons will be downloaded
    # simultaneously.

    # build filename
    eglibc="eglibc-${CT_LIBC_VERSION}.tar.bz2"
    eglibc_linuxthreads="${CT_LIBC}-linuxthreads-${CT_LIBC_VERSION}.tar.bz2"
    eglibc_localedef="${CT_LIBC}-localedef-${CT_LIBC_VERSION}.tar.bz2"
    eglibc_ports="${CT_LIBC}-ports-${CT_LIBC_VERSION}.tar.bz2"

    # Check if every tarballs are already present
    if [    -f "${CT_TARBALLS_DIR}/${eglibc}"                   \
         -a -f "${CT_TARBALLS_DIR}/${eglibc_linuxthreads}"      \
         -a -f "${CT_TARBALLS_DIR}/${eglibc_localedef}"         \
         -a -f "${CT_TARBALLS_DIR}/${eglibc_ports}"             \
       ]; then
        CT_DoLog DEBUG "Already have 'eglibc-${CT_LIBC_VERSION}'"
        return 0
    fi

    if [    -f "${CT_LOCAL_TARBALLS_DIR}/${eglibc}"                 \
         -a -f "${CT_LOCAL_TARBALLS_DIR}/${eglibc_linuxthreads}"    \
         -a -f "${CT_LOCAL_TARBALLS_DIR}/${eglibc_localedef}"       \
         -a -f "${CT_LOCAL_TARBALLS_DIR}/${eglibc_ports}"           \
         -a "${CT_FORCE_DOWNLOAD}" != "y"                           \
       ]; then
        CT_DoLog DEBUG "Got 'eglibc-${CT_LIBC_VERSION}' from local storage"
        for file in ${eglibc} ${eglibc_linuxthreads} ${eglibc_localedef} ${eglibc_ports}; do
            CT_DoExecLog ALL ln -s "${CT_LOCAL_TARBALLS_DIR}/${file}" "${CT_TARBALLS_DIR}/${file}"
        done
        return 0
    fi

    # Not found locally, try from the network
    CT_DoLog EXTRA "Retrieving 'eglibc-${CT_LIBC_VERSION}'"

    CT_MktempDir tmp_dir
    CT_Pushd "${tmp_dir}"

    do_eglibc_get
    CT_DoLog DEBUG "Moving 'eglibc-${CT_LIBC_VERSION}' to tarball directory"
    for file in ${eglibc} ${eglibc_linuxthreads} ${eglibc_localedef} ${eglibc_ports}; do
        CT_DoExecLog ALL mv -f "${file}" "${CT_TARBALLS_DIR}"
    done

    CT_Popd

    # Remove source files
    CT_DoExecLog ALL rm -rf "${tmp_dir}"

    if [ "${CT_SAVE_TARBALLS}" = "y" ]; then
        CT_DoLog EXTRA "Saving 'eglibc-${CT_LIBC_VERSION}' to local storage"
        for file in ${eglibc} ${eglibc_linuxthreads} ${eglibc_localedef} ${eglibc_ports}; do
            CT_DoExecLog ALL mv -f "${CT_TARBALLS_DIR}/${file}" "${CT_LOCAL_TARBALLS_DIR}"
            CT_DoExecLog ALL ln -s "${CT_LOCAL_TARBALLS_DIR}/${file}" "${CT_TARBALLS_DIR}/${file}"
        done
    fi

    return 0
}

# Extract eglibc
do_libc_extract() {
    CT_Extract "eglibc-${CT_LIBC_VERSION}"
    CT_Patch "eglibc" "${CT_LIBC_VERSION}"

    # C library addons
    for addon in $(do_libc_add_ons_list " "); do
        # NPTL addon is not to be extracted, in any case
        [ "${addon}" = "nptl" ] && continue || true
        CT_Pushd "${CT_SRC_DIR}/eglibc-${CT_LIBC_VERSION}"
        CT_Extract nochdir "eglibc-${addon}-${CT_LIBC_VERSION}"
        # Some addons have the 'long' name, while others have the
        # 'short' name, but patches are non-uniformly built with
        # either the 'long' or 'short' name, whatever the addons name
        # so we have to make symlinks from the existing to the missing
        # Fortunately for us, [ -d foo ], when foo is a symlink to a
        # directory, returns true!
        [ -d "${addon}" ] || ln -s "eglibc-${addon}-${CT_LIBC_VERSION}" "${addon}"
        [ -d "eglibc-${addon}-${CT_LIBC_VERSION}" ] || ln -s "${addon}" "eglibc-${addon}-${CT_LIBC_VERSION}"
        CT_Patch nochdir "eglibc" "${addon}-${CT_LIBC_VERSION}"
        CT_Popd
    done

    # The configure files may be older than the configure.in files
    # if using a snapshot (or even some tarballs). Fake them being
    # up to date.
    find "${CT_SRC_DIR}/eglibc-${CT_LIBC_VERSION}" -type f -name configure -exec touch {} \; 2>&1 |CT_DoLog ALL

    return 0
}

# Copy user provided eglibc configuration file if provided
do_libc_check_config() {
    if [ "${CT_EGLIBC_CUSTOM_CONFIG}" != "y" ]; then
        return 0
    fi

    CT_DoStep INFO "Checking C library configuration"

    CT_TestOrAbort "You did not provide an eglibc config file!" \
        -n "${CT_EGLIBC_OPTION_GROUPS_FILE}" -a \
        -f "${CT_EGLIBC_OPTION_GROUPS_FILE}"

    CT_DoExecLog ALL cp "${CT_EGLIBC_OPTION_GROUPS_FILE}" "${CT_CONFIG_DIR}/eglibc.config"

    # NSS configuration
    if grep -E '^OPTION_EGLIBC_NSSWITCH[[:space:]]*=[[:space:]]*n' "${CT_EGLIBC_OPTION_GROUPS_FILE}" >/dev/null 2>&1; then
        CT_DoLog DEBUG "Using fixed-configuration nsswitch facility"

        if [ "${CT_EGLIBC_BUNDLED_NSS_CONFIG}" = "y" ]; then
            nss_config="${CT_SRC_DIR}/eglibc-${CT_LIBC_VERSION}/nss/fixed-nsswitch.conf"
        else
            nss_config="${CT_EGLIBC_NSS_CONFIG_FILE}"
        fi
        CT_TestOrAbort "NSS config file not found!" -n "${nss_config}" -a -f "${nss_config}"

        CT_DoExecLog ALL cp "${nss_config}" "${CT_CONFIG_DIR}/nsswitch.config"
        echo "OPTION_EGLIBC_NSSWITCH_FIXED_CONFIG = ${CT_CONFIG_DIR}/nsswitch.config" \
            >> "${CT_CONFIG_DIR}/eglibc.config"

        if [ "${CT_EGLIBC_BUNDLED_NSS_FUNCTIONS}" = "y" ]; then
            nss_functions="${CT_SRC_DIR}/eglibc-${CT_LIBC_VERSION}/nss/fixed-nsswitch.functions"
        else
            nss_functions="${CT_EGLIBC_NSS_FUNCTIONS_FILE}"
        fi
        CT_TestOrAbort "NSS functions file not found!" -n "${nss_functions}" -a -f "${nss_functions}"

        CT_DoExecLog ALL cp "${nss_functions}" "${CT_CONFIG_DIR}/nsswitch.functions"
        echo "OPTION_EGLIBC_NSSWITCH_FIXED_FUNCTIONS = ${CT_CONFIG_DIR}/nsswitch.functions" \
            >> "${CT_CONFIG_DIR}/eglibc.config"
    else
        CT_DoLog DEBUG "Using full-blown nsswitch facility"
    fi

    CT_EndStep
}

# This function installs the eglibc headers needed to build the core compiler
do_libc_headers() {
    # Instead of doing two time the same actions, headers will
    # be installed with start files
    :
}

# Build and install start files
do_libc_start_files() {
    CT_DoStep INFO "Installing C library headers / start files"

    mkdir -p "${CT_BUILD_DIR}/build-libc-startfiles"
    cd "${CT_BUILD_DIR}/build-libc-startfiles"

    CT_DoLog EXTRA "Configuring C library"

    if [ "${CT_EGLIBC_CUSTOM_CONFIG}" = "y" ]; then
        CT_DoExecLog ALL cp "${CT_CONFIG_DIR}/eglibc.config" option-groups.config
    fi

    cross_cc=$(CT_Which "${CT_TARGET}-gcc")
    cross_cxx=$(CT_Which "${CT_TARGET}-g++")
    cross_ar=$(CT_Which "${CT_TARGET}-ar")
    cross_ranlib=$(CT_Which "${CT_TARGET}-ranlib")
    
    CT_DoLog DEBUG "Using gcc for target: '${cross_cc}'"
    CT_DoLog DEBUG "Using g++ for target: '${cross_cxx}'"
    CT_DoLog DEBUG "Using ar for target: '${cross_ar}'"
    CT_DoLog DEBUG "Using ranlib for target: '${cross_ranlib}'"

    BUILD_CC="${CT_BUILD}-gcc"                          \
    CC=${cross_cc}                                      \
    CXX=${cross_cxx}                                    \
    AR=${cross_ar}                                      \
    RANLIB=${cross_ranlib}                              \
    CT_DoExecLog CFG                                    \
    "${CT_SRC_DIR}/eglibc-${CT_LIBC_VERSION}/configure" \
        --prefix=/usr                                   \
        --with-headers="${CT_HEADERS_DIR}"              \
        --build="${CT_BUILD}"                           \
        --host="${CT_TARGET}"                           \
        --disable-profile                               \
        --without-gd                                    \
        --without-cvs                                   \
        --enable-add-ons

    CT_DoLog EXTRA "Installing C library headers"

    # use the 'install-headers' makefile target to install the
    # headers

    CT_DoExecLog ALL                    \
    make install-headers                \
         install_root=${CT_SYSROOT_DIR} \
         install-bootstrap-headers=yes

    CT_DoLog EXTRA "Installing C library start files"

    # there are a few object files needed to link shared libraries,
    # which we build and install by hand

    CT_DoExecLog ALL mkdir -p ${CT_SYSROOT_DIR}/usr/lib
    CT_DoExecLog ALL make csu/subdir_lib
    CT_DoExecLog ALL cp csu/crt1.o csu/crti.o csu/crtn.o \
        ${CT_SYSROOT_DIR}/usr/lib

    # Finally, 'libgcc_s.so' requires a 'libc.so' to link against.  
    # However, since we will never actually execute its code, 
    # it doesn't matter what it contains.  So, treating '/dev/null' 
    # as a C source file, we produce a dummy 'libc.so' in one step

    CT_DoExecLog ALL ${cross_cc} -nostdlib -nostartfiles -shared -x c /dev/null -o ${CT_SYSROOT_DIR}/usr/lib/libc.so

    CT_EndStep
}

# This function builds and install the full glibc
do_libc() {
    local -a extra_config

    CT_DoStep INFO "Installing C library"

    mkdir -p "${CT_BUILD_DIR}/build-libc"
    cd "${CT_BUILD_DIR}/build-libc"

    CT_DoLog EXTRA "Configuring C library"

    if [ "${CT_EGLIBC_CUSTOM_CONFIG}" = "y" ]; then
        CT_DoExecLog ALL cp "${CT_CONFIG_DIR}/eglibc.config" option-groups.config
    fi

    if [ "${CT_EGLIBC_OPT_SIZE}" = "y" ]; then
        OPTIMIZE=-Os
    else
        OPTIMIZE=-O2
    fi

    # Add some default glibc config options if not given by user.
    # We don't need to be conditional on wether the user did set different
    # values, as they CT_LIBC_GLIBC_EXTRA_CONFIG is passed after extra_config

    extra_config+=("--enable-kernel=$(echo ${CT_LIBC_GLIBC_MIN_KERNEL} |sed -r -e 's/^([^.]+\.[^.]+\.[^.]+)(|\.[^.]+)$/\1/;')")

    case "${CT_THREADS}" in
        nptl)           extra_config+=("--with-__thread" "--with-tls");;
        linuxthreads)   extra_config+=("--with-__thread" "--without-tls" "--without-nptl");;
        none)           extra_config+=("--without-__thread" "--without-nptl")
                        case "${CT_LIBC_GLIBC_EXTRA_CONFIG}" in
                            *-tls*) ;;
                            *) extra_config+=("--without-tls");;
                        esac
                        ;;
    esac

    case "${CT_SHARED_LIBS}" in
        y) extra_config+=("--enable-shared");;
        *) extra_config+=("--disable-shared");;
    esac

    case "${CT_ARCH_FLOAT_HW},${CT_ARCH_FLOAT_SW}" in
        y,) extra_config+=("--with-fp");;
        ,y) extra_config+=("--without-fp");;
    esac

    case "$(do_libc_add_ons_list ,)" in
        "") ;;
        *)  extra_config+=("--enable-add-ons=$(do_libc_add_ons_list ,)");;
    esac

    extra_cc_args="${extra_cc_args} ${CT_ARCH_ENDIAN_OPT}"

    cross_cc=$(CT_Which "${CT_TARGET}-gcc")    

    CT_DoLog DEBUG "Using gcc for target:     '${cross_cc}'"
    CT_DoLog DEBUG "Configuring with addons : '$(do_libc_add_ons_list ,)'"
    CT_DoLog DEBUG "Extra config args passed: '${extra_config[*]}'"
    CT_DoLog DEBUG "Extra CC args passed    : '${extra_cc_args}'"

    # ./configure is mislead by our tools override wrapper for bash
    # so just tell it where the real bash is _on_the_target_!
    # Notes:
    # - ${ac_cv_path_BASH_SHELL} is only used to set BASH_SHELL
    # - ${BASH_SHELL}            is only used to set BASH
    # - ${BASH}                  is only used to set the shebang
    #                            in two scripts to run on the target
    # So we can safely bypass bash detection at compile time.
    # Should this change in a future eglibc release, we'd better
    # directly mangle the generated scripts _after_ they get built,
    # or even after they get installed... eglibc is such a sucker...
    echo "ac_cv_path_BASH_SHELL=/bin/bash" >config.cache

    BUILD_CC="${CT_BUILD}-gcc"                                      \
    CFLAGS="${CT_TARGET_CFLAGS} ${CT_LIBC_GLIBC_EXTRA_CFLAGS} ${OPTIMIZE}"  \
    CC="${CT_TARGET}-gcc ${CT_LIBC_EXTRA_CC_ARGS} ${extra_cc_args}" \
    AR=${CT_TARGET}-ar                                              \
    RANLIB=${CT_TARGET}-ranlib                                      \
    CT_DoExecLog CFG                                                \
    "${CT_SRC_DIR}/eglibc-${CT_LIBC_VERSION}/configure"             \
        --prefix=/usr                                               \
        --with-headers="${CT_HEADERS_DIR}"                          \
        --build=${CT_BUILD}                                         \
        --host=${CT_TARGET}                                         \
        --disable-profile                                           \
        --without-gd                                                \
        --without-cvs                                               \
        --cache-file=config.cache                                   \
        "${extra_config[@]}"                                        \
        ${CT_LIBC_GLIBC_EXTRA_CONFIG}
    
    CT_DoLog EXTRA "Building C library"

    # eglibc build hacks
    # http://sourceware.org/ml/crossgcc/2008-10/msg00068.html
    case "${CT_ARCH},${CT_ARCH_CPU}" in
        powerpc,8??)
            CT_DoLog DEBUG "Activating support for memset on broken ppc-8xx (CPU15 erratum)"
            EGLIBC_BUILD_ASFLAGS="-DBROKEN_PPC_8xx_CPU15";;
    esac

    CT_DoExecLog ALL make ASFLAGS="${EGLIBC_BUILD_ASFLAGS}"

    CT_DoLog EXTRA "Installing C library"

    CT_DoExecLog ALL make install install_root="${CT_SYSROOT_DIR}"

    CT_EndStep
}

# This function finishes the glibc install
do_libc_finish() {
    # Nothing to be done for eglibc
    :
}

# Build up the addons list, separated with $1
do_libc_add_ons_list() {
    local sep="$1"
    local addons_list=$(echo "${CT_LIBC_ADDONS_LIST//,/${sep}}" |tr -s ,)
    case "${CT_THREADS}" in
        none)   ;;
        *)      addons_list="${addons_list}${sep}${CT_THREADS}";;
    esac
    [ "${CT_LIBC_GLIBC_USE_PORTS}" = "y" ] && addons_list="${addons_list}${sep}ports"
    addons_list="${addons_list%%${sep}}"
    echo "${addons_list##${sep}}"
}

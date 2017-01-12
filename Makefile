###############################################################################
# File Name  : Makefile                                                       #
# Description: local main project makefile                                    #
#              This makefile contains the project settings.                   #
# Application: GNU Make version 3.79.1, by Richard Stallman and Roland McGrath#
#              Built for Windows32                                            #
#                                                                             #
#              This Makefile includes:                                        #
#                 Makefile.project.part.defines                               #
#                 Makefile.XXX.YYY.ZZZ.make                                   #
#                 Global.Makefile.Target.make (includes <$(PROJECTNAME).dep>  #
#                                                                             #
#-----------------------------------------------------------------------------#
#               C O P Y R I G H T                                             #
#-----------------------------------------------------------------------------#
# Copyright (c) 2014 by Vector Informatik GmbH.  All rights reserved.         #
#                                                                             #
#-----------------------------------------------------------------------------#
#               A U T H O R   I D E N T I T Y                                 #
#-----------------------------------------------------------------------------#
# Initials     Name                      Company                              #
# --------     ---------------------     -------------------------------------#
# Pdr          Philipp Duller            Vector Informatik GmbH               #
# Csz          Carlos Sanchez            Vector Informatik GmbH               #
# Bwa          Benjamin Walter           Vector Informatik GmbH               #
#-----------------------------------------------------------------------------#
#               R E V I S I O N   H I S T O R Y                               #
#-----------------------------------------------------------------------------#
# Date         Ver    Sign Description                                        #
# ----------   -----  ---- ---------------------------------------------------#
# 2013/11/19   1.0.0  Pdr  Initial creation                                   #
#                          (adapted from zBrs_Sja2020@root[1.00.xx])          #
# 2014/01/29   1.1.0  Pdr  Patched - (now it is really a) Global GHS Makefile #
# 2014/03/26   1.2.0  Csz  added iMX6 comment within LDFLAGS_ADDITIONAL_POST  #
# 2014/11/18   2.0.0  Bwa  Adaptions for new BRS code structure               #
# 2015/01/08   2.0.1  Bwa  Adopted support for THUMB mode                     #
###############################################################################
#MKVERBOSE=1

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Version of MakeSupport used (current version is 3)
#------------------------------------------------------------------------------
VERSION     = 3
SUB_VERSION = 11

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Include the project specific settings
#------------------------------------------------------------------------------
include Makefile.config
include Makefile.project.part.defines
include Makefile.$(DERIVATIVE).definitions

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Name of this project (dir under which the appl dir resides)
# E.g.: TestSuit
#------------------------------------------------------------------------------
PROJECT_NAME = TestSuit

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# This mechanism has been canceled, so there's no need of changing anything.
#------------------------------------------------------------------------------
SUPPORTED_CPU = $(DERIVATIVE)
CPU_TYPE      = $(DERIVATIVE)

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Define Compiler path
# E.g.: COMPILER_BASE = D:\Uti\compiler\ARM\GHS\4_0_2\arm402
#       COMPILER_BIN  = $(COMPILER_BASE)\bin
#       COMPILER_INC  = $(COMPILER_BASE)\lib\hc08c\include
#       COMPILER_LIB  = $(COMPILER_BASE)\lib\hc08c\lib
#------------------------------------------------------------------------------
COMPILER_BASE = C:\GHS\comp_201416
COMPILER_BIN  = $(COMPILER_BASE)
COMPILER_INC  = $(COMPILER_BASE)\include\arm
COMPILER_LIB  = $(COMPILER_BASE)\lib\arm4

ifeq ($(OS_USECASE),)
#Default is BRS
OS_USECASE = BRS
endif

#------------------------------------------------------------------------------
# Default License server for the Greenhills Compiler (se209pcd <--> vistrlic1)
#------------------------------------------------------------------------------
LICENSE_SERVER = vistrlic1

#------------------------------------------------------------------------------
# Define Emulator path
# E.g.: EMU_PATH = C:\UTI\HITOPWIN\P6811
#------------------------------------------------------------------------------
EMU_PATH = $(COMPILER_BASE)

#------------------------------------------------------------------------------
# Define path to PCLint
# Default if it is not defined: PCLINT = c:\uti\lint\8.0\lint-nt
#------------------------------------------------------------------------------
PCLINT =

#------------------------------------------------------------------------------
# Define path to PCLint File
# Default if it is not defined: PCLINT_FILE = .\pclint.lnt
#------------------------------------------------------------------------------
PCLINT_FILE =

#------------------------------------------------------------------------------
# Define files excluded from PCLint processing
# E.g.:
# PCLINT_EXCL  = source/foo.c \
#                $(addprefix $(ROOT), $(call compSources,IL_VECTOR))
#------------------------------------------------------------------------------
PCLINT_EXCL  =

#------------------------------------------------------------------------------
# Set platform specific options for PCLINT
# E.g.:
# PCLINT_OPT_SPECIFIC = \
#                      -si2 \
#                      -sp2
#------------------------------------------------------------------------------
PCLINT_OPT_SPECIFIC = \
                      -e46                            \
                      -fcu                            \
                      -ss2                            \
                      -si4                            \
                      -sl4                            \
                      -sp4                            \
                      -A                              \
                      -elib(950)                      \
                      -wlib(1)                        \
                      +rw(__asm)                      \
                      +rw(__irq)                      \
                      +rw(__pure)                     \
                      +rw(__softfp)                   \
                      +rw(__swi)                      \
                      +rw(__swi_indirect)             \
                      +rw(__value_in_regs)            \
                      +rw(__inline)                   \
                      +rw(__weak)                     \
                      +rw(__int64)                    \
                      +rw(__global_reg)               \
                      +rw(__align)                    \
                      +rw(__packed)                   \
                      +rw(_to_brackets)               \
                      -d__swi=_to_brackets            \
                      -d__arm                         \
                      -d__ARMCC_VERSION               \
                      -d__APCS_INTERWORK              \
                      -d__APCS_ROPI                   \
                      -d__APCS_RWPI                   \
                      -d__APCS_SWST                   \
                      -d__BIG_ENDIAN                  \
                      -d__CC_ARM                      \
                      -d__DATE__                      \
                      -d__embedded_cplusplus          \
                      -d__FEATURE_SIGNED_CHAR         \
                      -d__FILE__                      \
                      -d__func__                      \
                      -d__LINE__                      \
                      -d__MODULE__                    \
                      -d__OPTIMISE_SPACE              \
                      -d__OPTIMISE_TIME               \
                      -d__prettyfunc__                \
                      -d__sizeof_int                  \
                      -d__sizeof_long                 \
                      -d__sizeof_ptr                  \
                      -d__SOFTFP__                    \
                      -d__STDC__                      \
                      -d__STDC_VERSION__              \
                      -d__STRTCT_ANSI__               \
                      -d__TARGET_ARCH                 \
                      -d__TARGET_CPU                  \
                      -d__TARGET_FEATURE_DOUBLEWORD   \
                      -d__TARGET_FEATURE_DSPMUL       \
                      -d__TARGET_FEATURE_HALFWORD     \
                      -d__TARGET_FEATURE_MULTIPLY     \
                      -d__TARGET_FEATURE_THUMB        \
                      -d__TARTGET_FPU                 \
                      -d__thumb                       \
                      -d__TIME__                      

#------------------------------------------------------------------------------
# Set defines for PCLINT (also used by GNU-Preprocessor)
# E.g.: PCLINT_DEF = CPU_FREQUENCY=$(CPU_FREQUENCY)
#------------------------------------------------------------------------------
PCLINT_DEF = BRS_DERIVATIVE_$(DERIVATIVE)                       \
             BRS_OSC_CLK=$(MAIN_OSC_CLK)                        \
             BRS_TIMEBASE_CLOCK=$(BRS_TIMEBASE_CLOCK)           \
             BRS_OS_USECASE_$(OS_USECASE)                       \
             BRS_EVA_BOARD_$(EVA_BOARD)                         \
             BRS_CPU_CORE_$(CPU_CORE)                           \
             BRS_PROGRAM_CODE_LOCATION_$(PROGRAM_CODE_LOCATION) \
             BRS_VECTOR_TABLE_LOCATION_$(VECTOR_TABLE_LOCATION) \
             BRS_PLATFORM_$(PLATFORM)                           \
             BRS_COMP_$(COMPILER_MANUFACTURER)

#------------------------------------------------------------------------------
# Set location of QAC
# E.g.: QAC_PATH = C:\Program Files\PRQA\QAC-6.0
#------------------------------------------------------------------------------
QAC_PATH = D:\uti\PRQA\QAC-7.0

#------------------------------------------------------------------------------
# Set defines for QAC Compiler Personality
#------------------------------------------------------------------------------
define QAC_PERSONALITY 

endef

#------------------------------------------------------------------------------
# The following define overrides the rule - how to assemble a file -
# implemented in Global.Makefile.target.make.
#------------------------------------------------------------------------------
#override define ASSEMBLE_RULE
#        $(ECHO) "Overriden AssembleRule assembling : $<"; \
#        $(AS) $(ASFLAGS) $(INCLUDES) $< $(REDIRECT_OUTPUT);
#endef

#------------------------------------------------------------------------------
# The following define overrides the rule - how to compile a file -
# implemented in Global.Makefile.target.make.
#------------------------------------------------------------------------------
override define COMPILE_RULE
        if [ -n "$(MKVERBOSE)" ] ; then                                    \
                $(ECHO) "    $(CC) $(CFLAGS) $(INCLUDES) $(subst /,\,$<)"; \
        fi;                                                                \
        $(CC_ENV)                                                          \
        $(subst \,/,$(CC) $(CFLAGS)) $(INCLUDES_UNIX) $< 2> $(ERR_PATH)/$*.$(ERR_SUFFIX);
endef

#------------------------------------------------------------------------------
# The following define overrides the rule - how to link the object files -
# implemented in Global.Makefile.target.make.
#------------------------------------------------------------------------------
override define LINK_RULE
        if [ -n "$(MKVERBOSE)" ] ; then                    \
                $(ECHO) "    $(LD) $(OBJECTS) $(LDFLAGS)"; \
        fi;                                                \
        $(LD_ENV)                                          \
        $(subst \,/,$(LD) $(OBJECTS) $(LDFLAGS)) 2> $(ERR_PATH)/$*.$(ERR_SUFFIX);
endef

#------------------------------------------------------------------------------
# The following define overrides the rule - how to build a lib file -
# implemented in Global.Makefile.target.make.
#------------------------------------------------------------------------------
#override define LIB_RULE
#  if [ -n "$(MKVERBOSE)" ] ; then                                        \
#    $(ECHO) '    $(LB_U) $(LBFLAGS) $(subst \,/,$^) $(REDIRECT_OUTPUT)'; \
#  fi;                                                                    \
#  $(LB_ENV_U) $(LB_U) $(subst \,/,$^) $(LBFLAGS) $(REDIRECT_OUTPUT);
#endef

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#                   This is a list of all -cpu <options>                    #
#             Point of Time: MULTI v6.1.4 - Compiler v2013.1.4              #
#                           Possible values are:                            #
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
# arm6, arm7, arm7m, arm7tm, arm8, arm9, arm9e, arm10, microRAD, strongarm, #
#   xscale, arm11, arm11t2, cortexa8, cortexa9, cortexm3, cortexr4, pj4,    #
#      pj4v6, cortexm0, cortexm0plus, cortexm1, cortexm4, cortexm4f,        #
#      cortexr4f, cortexr5, cortexr7, cortexa5, cortexa7, cortexa15         #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
CPU_SWITCH_ARM7       = arm7tm
CPU_SWITCH_ARM9       = arm9
CPU_SWITCH_ARM11      = arm11
CPU_SWITCH_CORTEX_M0  = cortexm0
CPU_SWITCH_CORTEX_M3  = cortexm3
CPU_SWITCH_CORTEX_M4  = cortexm4
CPU_SWITCH_CORTEX_R4  = cortexr4
CPU_SWITCH_CORTEX_A5  = cortexa5
CPU_SWITCH_CORTEX_A7  = cortexa7
CPU_SWITCH_CORTEX_A8  = cortexa8
CPU_SWITCH_CORTEX_A9  = cortexa9
CPU_SWITCH_CORTEX_A15 = cortexa15
CPU_SWITCH_CORTEX_R5  = cortexr5


ifeq ($(CPU_SWITCH_$(CPU_CORE)),)
 $(warning COMPILERFLAGERROR: The compiler option (-cpu=<value>) is not defined!)
endif

################################################################################
#################### all below this line MUST be defined  ######################
################################################################################

#------------------------------------------------------------------------------
# make builds the whole project by default
# To make another target you have to add it to the prerequisites of default.
# E.g.: default: all $(PROJECT_NAME).opt
#------------------------------------------------------------------------------
default: all

#------------------------------------------------------------------------------
# If platform specific rules exist, they can be described here to be shown
# in the usage output of the help target
# E.g.: PLATFORM_HELP = \
#           $(ECHO) "m mytarget ................ -- build mytarget"; \
#           $(ECHO) "                               nextline";
#------------------------------------------------------------------------------
PLATFORM_HELP =

###############################################################################
# Platform/compiler/derivative/emulator are relevant for the include of       #
# linker and symbol preprocessor command file identified by name.             #
# Vector standard names have to be used                                       #
###############################################################################
#------------------------------------------------------------------------------
# Compiler Manufacturer used in this project
# E.g.: ARM, COSMIC, DIABDATA, MICROTEC, TASKING, FUJITSU, GHS, NEC, GNU, GAIO
#       HIWARE, IAR, KEIL, MITSUBISHI, OKI, HEW, TI, GHS, TOSHIBA, DIABDATA, 
#       RENESAS
#       ALL is special for "all available compilers are supported in this file"
#------------------------------------------------------------------------------
COMPILER_MANUFACTURER = GHS

#------------------------------------------------------------------------------
# Platform used for this project
# E.g.: 78K0, AT91, C16X, C5x5, CANOE, CR16, FJ16LX, H8S, HC08, HC12, M16C
#       M32C, M32R, MC68376, MGT5100, MSM9225, PIC, PPC, SH7055, ST7, TLC900
#       TMS470, V850-DCAN, V850X, XC16X
#------------------------------------------------------------------------------
PLATFORM = ARM

#------------------------------------------------------------------------------
# Emulator used for this project
# E.g.: HITEX, LAUTERBACH, ISYSTEMS, GHS
#       ALL is special for "all derivatives are supported in this file"
#------------------------------------------------------------------------------
EMULATOR = ALL

#------------------------------------------------------------------------------
# $(AS_ENV) is the environment necessary to run the assembler
# $(AS) defines the path to the assembler
# $(ASFLAGS_ADDITIONAL_PRE) defines all additional assembler switches that will
#                           be placed in front of all other switches
# $(ASFLAGS_WARNING) defines the assembler switches concerning warning output
# $(ASFLAGS_OPTIMIZATION) defines the assembler switches concerning optimization
# $(ASFLAGS_MEMORYMODEL) defines the assembler switches concerning the memory model
# $(ASFLAGS_ADDITIONAL_POST) defines all additional assembler switches that will be
#                            placed at the end of all switches
# E.g.: AS_ENV  = export AS_BIN="$(COMPILER_BIN)"
#       AS      = $(COMPILER_BIN)\Ahc08.exe
#------------------------------------------------------------------------------
AS_ENV  = $(LD_ENV)
ifeq ($(INSTRUCTION_SET),THUMB)
AS      = $(COMPILER_BIN)\ccthumb.exe
else
AS      = $(COMPILER_BIN)\asarm.exe
endif

ASFLAGS_ADDITIONAL_PRE =
ASFLAGS_WARNING        =
ASFLAGS_OPTIMIZATION   =
ASFLAGS_MEMORYMODEL    =

ASFLAGS_ADDITIONAL_POST = $(ASFLAGS_VECTOR_MAKESUPPORT) \
                          $(ASFLAGS_VECTOR_OPTIONS)     \
                          $(ASFLAGS_CUSTOMER_OPTIONS)

#Please add only absolutely mandatory options below (options which are needed by the makesupport)
ASFLAGS_VECTOR_MAKESUPPORT = -o $(OBJ_PATH)/$*.$(OBJ_SUFFIX) \
                             -stderr $(ERR_PATH)\$*.err

#Please add any other option below (vector default options)
ASFLAGS_VECTOR_OPTIONS = -cpu=$(CPU_SWITCH_$(CPU_CORE)) \
                         -I $(COMPILER_INC)             \
                         -fpu=none                      \
                         -b0                            \
                         -armuk                         \
                         -dwarf2                        \
                         -g

#Please enter all customer options below and remove ASFLAGS_VECTOR_OPTIONS from ASFLAGS_ADDITIONAL_POST
ASFLAGS_CUSTOMER_OPTIONS =

#------------------------------------------------------------------------------
# $(ASVERSION) defines the assembler switch for version report
# E.g.: ASVERSION = -V
#------------------------------------------------------------------------------
ASVERSION = -V

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# $(CC_ENV) is the environment necessary to run the compiler
# $(CC) defines the path to the C-Compiler
# $(CFLAGS_ADDITIONAL_PRE) defines all additional compiler switches that will
#                          be placed in front of all other switches
# $(CFLAGS_WARNING) defines the compiler switches concerning warning output
# $(CFLAGS_OPTIMIZATION) defines the compiler switches concerning optimization
# $(CFLAGS_MEMORYMODEL) defines the compiler switches concerning the memory model
# $(CFLAGS_ADDITIONAL_POST) defines all additional compiler switches that will be
#                           placed at the end of all switches
# It is highly recommended to add the compiler switch description in a comment
# E.g.: CC_ENV  = export CC_BIN="$(COMPILER_BIN)"
#       CC       = $(COMPILER_BIN)\cx6808.exe
#------------------------------------------------------------------------------
# Options: (Not all options apply to all languages or targets).
# -a                Compile for code coverage analysis by Multi.
# -archive          Create a library archive for use with linker.
# --arm             Set the C/C++ Compiler Language Dialect option to Standard C
# -asm=<args>       Pass <args> directly to the assembler.
# -[no]bigswitch    Allow large switch statements.
# -c                Produce object files, but do not link.
# -check            Run-Time error checking. Possible Parameters:
#                   --> none,all,[no]bound,[no]nilderef,[no]switch,[no]assignbound,[no]zerodivide. 
#                   The compiler generates runtime checks for the items requested.
# -cpu              To specify a target processor with the driver, pass the -cpu=cpu option.
# -D<name>[=<val>]  In C, C++: define <name> with value <val>. (Preprocessor-MACRO)
# --diag_suppress   Sets the specified diagnostics message to the level of silent.
# -dual_debug       DWARF debugging information
# -dwarf2           Enables the generation of DWARF debugging information in the object file 
#                   (in addition to the Green Hills .dbo format)
# -E                In C, C++: preprocess file and send result to stdout.
# -entry=symbol     Set the entry point in the linker output.
# -farcalls         This option instructs the compiler to generate a far function call for every call
#                     An ARM mode function call is normally performed with an instruction that contains
#                     a 24-bit value that is shifted left by 2 and is added to the address of the current
#                     instruction to yield the address of the called function.
# -fsoft            Use software floating point.
# -fnone            In C, C++: Give syntax errors for floating point usage
# -G                Generate information for MULTI debugger.
# -H                Print names of included headers to stderr.
# -I<dir>           Passed to compiler to add <dir> to include search path.
# -L<dir>           Passed to linker to add <dir> to library search path.
# -l<name>          Passed to linker to look for library lib<name>.a.
# --long_long       [default] Support the long long data type.
# -lnk=<arg>        Pass <arg> directly to linker.
# -list[=name]      Assembler will produce a listing file.
# -list/<type>      For Ada95, generate the requested listing.
# -map[=name]       Linker will produce a map file.
# -nofloatio        Use printf, scanf without %e %f %g in libansi.
# -nostartfiles     Do not add start-up files to link
# -no_misalign_pack  The compiler generates extra code to handle misaligned data accesses.
# -nostdlib         Do not add start-up files or libraries to link
# -o name           Name final output file.
# -Ospeed           Optimize for speed, even if code is larger.
# -Ospace           Optimize for smaller code, even if it is slower.
# -O                Turn on general optimizations
# -object_dir="my"  Put object files in ./my
# -OI               Expand routines inline rather than generating calls.
# -OI=name,...      Inline only the named routines.
# -OL               Optimize loops, including loop unrolling.
# -Onopeep          Turn off peephole optimization.
# -P                In C, C++: preprocess into file.i and stop.
# -p                Compile for call count profiling.
# -pg               Compile for call count and call graph profiling.
# -passsource       Pass compiler source lines into assembly file.
# -S                Produce assembly files, and stop.
# -syntax           Compilers will check syntax but not generate code.
# -thumb            Enables Thumb mode. This option is the default for Thumb-only processors (Cortex M family)
# -U<name>          In C, C++: undefine the macro <name>.
# -v                Print all commands before they are executed.
# -w                Suppress compiler, assembler, and linker warning messages.
# -#                Print all commands INSTEAD of executing them.
#------------------------------------------------------------------------------
CC_ENV = export GHS_LMHOST=@$(LICENSE_SERVER); \
         export GHS_LMWHICH="ghs";
ifeq ($(INSTRUCTION_SET),THUMB)
CC       = $(COMPILER_BIN)\ccthumb.exe
else
CC     = $(COMPILER_BIN)\ccarm.exe
endif

CFLAGS_ADDITIONAL_PRE  =
CFLAGS_WARNING         =
CFLAGS_OPTIMIZATION    = -Onone
CFLAGS_MEMORYMODEL     =

CFLAGS_ADDITIONAL_POST = $(CFLAGS_VECTOR_MAKESUPPORT) \
                         $(CFLAGS_VECTOR_OPTIONS)     \
                         $(CFLAGS_CUSTOMER_OPTIONS)

#Please add only absolutely mandatory options below (options which are needed by the makesupport)
CFLAGS_VECTOR_MAKESUPPORT = -DBRS_TIMEBASE_CLOCK=$(BRS_TIMEBASE_CLOCK)           \
                            -DBRS_OSC_CLK=$(MAIN_OSC_CLK)                        \
                            -DBRS_EVA_BOARD_$(EVA_BOARD)                         \
                            -DBRS_DERIVATIVE_$(DERIVATIVE)                       \
                            -DBRS_OS_USECASE_$(OS_USECASE)                       \
                            -DBRS_PROGRAM_CODE_LOCATION_$(PROGRAM_CODE_LOCATION) \
                            -DBRS_VECTOR_TABLE_LOCATION_$(VECTOR_TABLE_LOCATION) \
                            -DBRS_CPU_CORE_$(CPU_CORE)                           \
                            -DBRS_STACK_SIZE=$(STACKSIZE)                        \
                            -DBRS_PLATFORM_$(PLATFORM)                           \
                            -DBRS_COMP_$(COMPILER_MANUFACTURER)                  \
                            -object_dir=$(OBJ_PATH)                              \
                            -list=$(LST_PATH)/$*.lst

#Please add any other option below (vector default options)
CFLAGS_VECTOR_OPTIONS = -cpu=$(CPU_SWITCH_$(CPU_CORE)) \
                        -noobj                         \
                        --long_long                    \
                        -G                             \
                        -dual_debug                    \
                        -c                             \
                        -farcalls                      \
                        -dwarf2                        \
                        -no_misalign_pack              \
                        -nostartfiles                  \
                        -pragma_asm_inline             \
                        $(CFLAGS_$(INSTRUCTION_SET))

#Please enter all customer options below and remove CFLAGS_VECTOR_OPTIONS from CFLAGS_ADDITIONAL_POST
CFLAGS_CUSTOMER_OPTIONS =

CFLAGS_ARM   = --arm
CFLAGS_THUMB = -thumb

#------------------------------------------------------------------------------
# $(CVERSION) defines the compiler switch for version report
# E.g.: CVERSION = -V
#------------------------------------------------------------------------------
CVERSION = -V dummy.c

#------------------------------------------------------------------------------
# $(CC_LC) defines the number of lines used for compiler version information 
# in brsvinfo.h
#------------------------------------------------------------------------------
CC_LC = 6

#------------------------------------------------------------------------------
# $(CINC) defines the include switch of the used compiler.
# E.g.: COSMIC Compiler use: CINC = -i
#------------------------------------------------------------------------------
CINC = -I

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# $(LD_ENV) is the environment necessary to run the linker
# $(LD) defines the path linker
# $(LDFLAGS_ADDITIONAL_PRE) defines all additional linker switches that will
#                           be placed in front of all other switches
# $(LDFLAGS_WARNING) defines the linker switches concerning warning output
# $(LDFLAGS_OPTIMIZATION) defines the linker switches concerning optimization
# $(LDFLAGS_MEMORYMODEL) defines the linker switches concerning the memory model
# $(LDFLAGS_ADDITIONAL_POST) defines all additional linker switches that will be
#                            placed at the end of all switches
# It is highly recommended to add the compiler switch description in a comment
# E.g.: LD_ENV  = export LD_BIN="$(COMPILER_BIN)"
#       LD      = $(COMPILER_BIN)\clnk.exe
#------------------------------------------------------------------------------
# -Bstatic 		    Do not link against shared libraries.
# -cpu            To specify a target processor with the driver, pass the -cpu=cpu option.
# -dual_debug     Enables the generation of DWARF, COFF, or BSD debugging information in
#                 the object file (in addition to the Green Hills .dbo format),
#                 according to the convention for your target
# -e=<symbol>     Change entry point to <symbol>.
# -G              Generate information for MULTI debugger.
# -hex             Generates output file in HEX386 format with .run extension
# -keepmap        Controls the retention (dt. Aufrechterhaltung) of the map file in the event of a link error
# -Map=<mapfile>	Print a link map to the file mapfile.		
# -nostartfiles   Do Not Use Start Files 
# -o <file> 		  Place output in file <file>.
# -T <scriptfile>	Use scriptfi le as the linker script.
#------------------------------------------------------------------------------
LD_ENV = $(CC_ENV)
LD     = $(CC)

LDFLAGS_ADDITIONAL_PRE  =
LDFLAGS_WARNING         =
LDFLAGS_OPTIMIZATION    =
LDFLAGS_MEMORYMODEL     =

LDFLAGS_ADDITIONAL_POST = $(LDFLAGS_VECTOR_MAKESUPPORT) \
                          $(LDFLAGS_VECTOR_OPTIONS)     \
                          $(LDFLAGS_CUSTOMER_OPTIONS)

#Please add only absolutely mandatory options below (options which are needed by the makesupport)
LDFLAGS_VECTOR_MAKESUPPORT =

#Please add any other option below (vector default options)
LDFLAGS_VECTOR_OPTIONS = -o $*.$(BINARY_SUFFIX)            \
                          -nostartfiles                     \
                          -pragma_asm_inline                \
                          -map=$(PROJECT_NAME).map          \
                          -g                                \
                          -keepmap                          \
                          -dual_debug                       \
                          -hex                              \
                          $(PROJECT_NAME).$(LNK_SUFFIX)

#Please enter all customer options below and remove LDFLAGS_VECTOR_OPTIONS from LDFLAGS_ADDITIONAL_POST
LDFLAGS_CUSTOMER_OPTIONS =

ifeq ($(CPU_CORE),$(filter $(CPU_CORE),CORTEX_M0 CORTEX_M3 CORTEX_M4))
 ##########################################################################################
 # If you're using a Cortex-M, the 1st Inst. might be a "PUSH", but you don't have        #
 # an initialized stackpointer (sp). So you'll have to start @ "Startup_Handler+2",       #
 # where "Startup_Handler+2" is an absolute address, which can be taken from the MAP-File #
 ##########################################################################################
 LDFLAGS_ADDITIONAL_POST += -e=0x________
 #For iMX6, also this code was sufficient:
 #LDFLAGS_ADDITIONAL_POST += -e=Startup_Handler
else
 LDFLAGS_ADDITIONAL_POST += -e=Startup_Handler
endif

ifneq ($(CPU_SWITCH_$(CPU_CORE)),)
 LDFLAGS_ADDITIONAL_POST += -cpu=$(CPU_SWITCH_$(CPU_CORE))
endif
#------------------------------------------------------------------------------
# $(LDVERSION) defines the linker switch for version report
# E.g.: LDVERSION = -v
#------------------------------------------------------------------------------
LDVERSION = $(CVERSION)

#------------------------------------------------------------------------------
# $(LD_LC) defines the number of lines used for linker version information 
# in brsvinfo.h
#------------------------------------------------------------------------------
LD_LC = $(CC_LC)

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# $(LB_ENV) is the environment necessary to run the librarian
# $(LB) defines the path to the C-librarian
# $(LBFLAGS) defines all librarian switches
# It is highly recommended to add the librarian switch description in a comment
# E.g.: LB_ENV  = export LB_BIN="$(COMPILER_BIN)";
#       LB      = $(COMPILER_BIN)\cx6808.exe
#------------------------------------------------------------------------------
# Invoce ccarm.exe with option -archive.
# Modifying Libraries with Advanced Options
# :archive.args=-options
# 
# c Suppresses warnings when creating a library that did not exist.
# C Ignores the existing library.
# e Prefixes messages with ERROR or WARNING. Equivalent to the
#   compiler driver option -prefixed_msgs.
# o Keeps original timestamps when extracting files from a library.
# s Regenerates the table of contents; use with -t.
# S Prevents the creation of a table of contents.
# v Prints verbose messages when executing a command.
#------------------------------------------------------------------------------
LB_ENV  = $(LD_ENV)
LB      = $(LD)
LBFLAGS = :archiver.args=-c \
          -archive          \
          -o $@

#------------------------------------------------------------------------------
# $(LBVERSION) defines the archiver switch for version report
# E.g.: LBVERSION = -v
#------------------------------------------------------------------------------
LBVERSION = $(LDVERSION)
LB_LC     = $(LD_LC)

#------------------------------------------------------------------------------
# $(OBJ_OUTPUT) defines the way, how to generate error files
# It should be one of:
#     MOVE  - Object files are generated by compiler/linker and will be moved
#             to $(OBJ_PATH) after generation by external tool
#     FLAGS - Object file path definition is supported by compiler.
#             Therefore the $(CFLAGS) have to be adjusted.
#------------------------------------------------------------------------------
OBJ_OUTPUT = FLAGS

#------------------------------------------------------------------------------
# $(ERR_OUTPUT) defines the way, how to generate error files
# It should be one of:
#     MOVE  - Error files are generated by compiler/linker and will be moved
#             to $(ERR_PATH) after generation by external tool
#     PIPE  - Error will be printed to sterr/stdout and redirected to
#             $(ERR_PATH)
#     FLAGS - Error file generation with path definition is supported by
#             compiler. Therefore e.g. the $(CFLAGS) have to be adjusted.
#------------------------------------------------------------------------------
ERR_OUTPUT = PIPE

#------------------------------------------------------------------------------
# $(LIB_OUTPUT) defines the way, how to generate lib files
# It should be one of:
#     MOVE  - Lib files are generated by compiler/linker and will be moved
#             to $(LIB_PATH) after generation.
#     FLAGS - Object file path definition is supported by compiler.
#             Therefore the $(LB_FLAGS) have to be adjusted.
#------------------------------------------------------------------------------
LIB_OUTPUT = FLAGS

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Suffix of generated object files. Generated objects are then *.$(OBJ_SUFFIX)
#------------------------------------------------------------------------------
OBJ_SUFFIX = o

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Suffix of assembler files.
#------------------------------------------------------------------------------
ASM_SUFFIX = asm

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Suffix of generated list files. Generated list files are then *.$(LST_SUFFIX)
#------------------------------------------------------------------------------
LST_SUFFIX = lst

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Suffix linker command file.
#------------------------------------------------------------------------------
LNK_SUFFIX = ld

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Suffix binary output file.
#------------------------------------------------------------------------------
BINARY_SUFFIX = bin

#------------------------------------------------------------------------------
#------------------------- MUST be filled out ---------------------------------
# Suffix of generated library files. Generated objects are then *.$(LIB_SUFFIX)
#------------------------------------------------------------------------------
LIB_SUFFIX = lib

#------------------------------------------------------------------------------
# Suffixes of generated error files which are placed in $(ERR_PATH)
# (necessary for Visual Studio / Codewright project file generation)
# Default (nothing entered) is err
# E.g. ERR_SUFFIX_LIST = err
#------------------------------------------------------------------------------
ERR_SUFFIX_LIST = err

#------------------------------------------------------------------------------
# Suffixes of generated asm/list files which are placed in $(LST_PATH)
# (necessary for Visual Studio / Codewright project file generation)
# Default (nothing entered) is lst
# E.g. LST_SUFFIX_LIST = lst asm
#------------------------------------------------------------------------------
LST_SUFFIX_LIST = lst

#------------------------------------------------------------------------------
# Location of v_cfg.h relative to the Makefiles
# This is needed to retrive $(CPU_TYPE) for derivative control
# E.g.: $(GENDATA_DIR)
#------------------------------------------------------------------------------
V_CFG_LOC = $(GENDATA_DIR)

#------------------------------------------------------------------------------
# List of (regexp) pattern used by the distclean target
# Pattern. deleted by default are:
#    *~
#    *.*~
#    *.~*
#    *.bak
#    *.log
#    $(GENDATA_DIR)\*.txt
#    $(ERR_LIST_WOP)
#    $(LST_N_MAP_LIST_WOP)
#    $(SIMPLEOS_OIL)_
#    $(PROJECT_NAME).map
#    $(PROJECT_NAME).dep
#    $(PROJECT_NAME).dsw
#    $(PROJECT_NAME).dsp
#    $(PROJECT_NAME).ncb
#    $(PROJECT_NAME).opt
#    $(PROJECT_NAME).plg
#    $(PROJECT_NAME).doc.cfg
#    $(GENDATA_DIR)\BrsVInfo.h
#------------------------------------------------------------------------------
DIST_CLEAN_PATTERN_LIST = $(PROJECT_NAME).abs.plg *.bak *.Bak *.Uv2 misra *.prj \
                          *.p_a *.p_c *.p_s *.lnk

#------------------------------------------------------------------------------
# Additional includes essentially for compilation
#------------------------------------------------------------------------------
ADDITIONAL_INCLUDES +=

#------------------------------------------------------------------------------
# Additional objects essentially for linking
# E.g.: ADDITIONAL_OBJECTS = $(OBJ_PATH)\myobject.$(OBJ_SUFFIX)
#------------------------------------------------------------------------------
ADDITIONAL_OBJECTS +=

#------------------------------------------------------------------------------
# List of assembler source files
# E.g.: ASM_SOURCES = source\startup.$(ASM_SUFFIX)
#------------------------------------------------------------------------------
ASM_SOURCES +=

#------------------------------------------------------------------------------
# Add Startup code to application source list
# E.g.: APP_SOURCE_LST += source\startup.c
#------------------------------------------------------------------------------
APP_SOURCE_LST +=

#------------------------------------------------------------------------------
# Dependency defines
# If compiler specific defines are questioned in platform specific files they
# can be manually defined here for the dependencies generation
# E.g.:
#------------------------------------------------------------------------------
DEP_DEFINES = $(PCLINT_DEF)

#------------------------------------------------------------------------------
# Turn strict c setting for dependency generation off
# E.g.: STRICT_C_OFF =   (ON)
#       STRICT_C_OFF = 1 (OFF)
#------------------------------------------------------------------------------
STRICT_C_OFF =

###############################################################################
# resource information generation
###############################################################################

#------------------------------------------------------------------------------
# Select the source, which contains the memory information
# E.g.: RESOURCE_FILE = LIST
#       RESOURCE_FILE = MAP
#       RESOURCE_FILE = NONE
#------------------------------------------------------------------------------
RESOURCE_FILE = MAP

#------------------------------------------------------------------------------
# The extension of the map or list file.
# E.g.: RESOURCE_FILE_EXT = map
#       RESOURCE_FILE_EXT = lst
#------------------------------------------------------------------------------
RESOURCE_FILE_EXT = map

#------------------------------------------------------------------------------
# This is the main resource script written in the language AWK.
# For more information about AWK see the following resources:
#   http://torvalds.cs.mtsu.edu/~neal/awkcard.pdf (recommendable)
#   http://www.uni-magdeburg.de/urzs/awk/
#   http://de.wikipedia.org/wiki/AWK
# Note: The character '$' must be escaped with a second '$'. So write '$$'.
#       Also the character '#' must be escaped using a backslash -> \#.
# For a more detailed description about what is going on in this script look
# at the file:  fetch-memory-info-template-documented.awk
# In order to see what the ressource scanner is doing it is highly recommended
# to set the environment variable GAWK_DEBUG=1. This option will generate a
# further file ($(PROJECT_NAME)_resource_debug.txt) in your log directory.
# Here is a short description about the instructions which has to be configured:
# RS means Record Separator and is set to a char which is used to split the
#   files into records. For each record the processing instructions are done.
#   The record string is assigned to the variable $0 (remember to access via $$0).
# FS means Field Separator and splits each record into fields. These fields
#   are referenced from $1, $2... $NF, where NF is the number of fields.
#   If FIELDWIDTHS is set, the value of FS is ignored.
# FIELDWIDTHS is a space separated list, each field is expected to have a fixed
#   width. The value of FS is ignored.
# IGNORECASE is a bool variable. When set to 1 comparsions are made case-
#   insensitive in comparsions with '==' and regular expressions.
# memoryTable maps the second argument of addEntry() to the according section.
# startProcessing() and stopProcessing() implements a little restriction
#   to which section of the file the instructions are executed.
# addEntry(module, memory, size, isHex) is the main API function you have to
#   use. It adds an entry to the internal array. You specify with the param.
#   how many bytes (param:size) to which module (param:module) and to which
#   type of memory (param:memory) you want to add. If you set the param
#   isHex to 1 the param size is treated as being hexadecimal.
# Other API functions are: trim(str), getStringWithinParentheses(str),
#   getStringWithinBrackets(str)
#------------------------------------------------------------------------------
RESOURCE_FETCH_SCRIPT = BEGIN {                                             \
  RS  = "\n";                                                               \
  FS  = "[ \\+]+";                                                          \
  IGNORECASE = 1;                                                           \
  memoryTable["ram"]    = "bss";                                            \
  memoryTable["iram"]   = "data";                                           \
  memoryTable["const"]  = "rodata|intvect";                                 \
  memoryTable["code"]   = "text";                                           \
  memoryTable["ignore"] = "debug|syscall|fixtype|fixaddr|interfunc|org";    \
  memoryTableOrder      = "const,code,ram,iram,ignore";                     \
}                                                                           \
                                                                            \
/^Module Summary/    { startProcessing() }                                  \
/^Global Symbols/    { stopProcessing() }                                   \
                     { process() }                                          \
                                                                            \
$$2 ~ /^[0-9a-f]+$$/ {                                                      \
  addEntry($$NF, $$3, $$2, 1);                                              \
}

###############################################################################
######### DO NOT remove these lines from the end of the Makefile!!! ###########
###############################################################################

#------------------------------------------------------------------------------
# Check if all necessary variables are set
#------------------------------------------------------------------------------
ifeq ($(MAKESUPPORT_DIR),)
    $(error MAKESUPPORT_DIR not set in m.bat or m.bat not called)
endif

ifeq ($(PLATFORM),)
    $(error Variable PLATFORM is not defined)
endif

ifeq ($(COMPILER_MANUFACTURER),)
    $(error Variable COMPILER_MANUFACTURER is not defined)
endif

ifeq ($(EMULATOR),)
    $(error Variable EMULATOR is not defined)
endif

ifeq ($(VERSION),)
    $(error Variable VERSION is not defined)
endif

ifeq ($(MAKESUPPORT_DIR),)
    $(error Variable MAKESUPPORT_DIR is not defined)
endif

#------------------------------------------------------------------------------
# List of variables in Makefile.config which are required and must be set by
# user. Check is performed by global makefile
#------------------------------------------------------------------------------
REQUIRED_MK_CONF_VARS = BRS_TIMEBASE_CLOCK    \
                        MAIN_OSC_CLK          \
                        DERIVATIVE            \
                        CPU_CORE              \
                        STACKSIZE             \
                        INSTRUCTION_SET       \
                        PROGRAM_CODE_LOCATION \
                        VECTOR_TABLE_LOCATION

#------------------------------------------------------------------------------
# Platform/compiler/derivative/emulator dependant makefile is included here
# It defines a rule to generate the linker and a rule to generate the
# symbol preprocessor command file.
# A template for this Makefile can be found under
#     R:\Can_Base\CANtate\BRS\PlatformSpecific\...\OrganiAndMake\...
#------------------------------------------------------------------------------
include Makefile.$(PLATFORM).$(COMPILER_MANUFACTURER).$(EMULATOR).make

#------------------------------------------------------------------------------
# Include the main makefile where all the targets are defined
#------------------------------------------------------------------------------
include $(MAKESUPPORT_DIR)\Global.Makefile.target.make.$(VERSION)

# End of Makefile

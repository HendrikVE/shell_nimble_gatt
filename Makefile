# name of your application
APPLICATION = shell_nimble_gatt

# If no BOARD is found in the environment, use this default:
BOARD ?= nrf52dk

# So far, NimBLE only works on nRF52 based platforms
BOARD_WHITELIST := nrf52dk nrf52840dk

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Include NimBLE
USEPKG += nimble
USEMODULE += nimble_svc_gap
USEMODULE += nimble_svc_gatt

# We also use the AD part of the BLE helper module
USEMODULE += bluetil_ad

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include

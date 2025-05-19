SG_BOOT ?= sg_boot
SOC     ?= SG200X

run: run_bt0 load_main load_payload

run_bt0:
	$(MAKE) -C bt0 run

load_main:
	$(MAKE) -C main load

load_payload:
	sleep 1
	$(SG_BOOT) load $(PAYLOAD)
	sleep 1
	$(SG_BOOT) load $(DTB)

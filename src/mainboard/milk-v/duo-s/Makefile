SG_BOOT ?= sg_boot
SOC     ?= SG200X

run: run_bt0 run_main load_payload

run_bt0:
	$(MAKE) -C bt0 run

run_main:
	$(MAKE) -C main run

load_payload:
	$(SG_BOOT) run $(PAYLOAD)
	$(SG_BOOT) run $(DTB)

SG_BOOT ?= sg_boot
SOC     ?= SG200X

run: run_bt0 run_main run_payload

run_bt0:
	$(MAKE) -C bt0 run

run_main:
	$(MAKE) -C main run

run_payload:
	$(SG_BOOT) run --main $(PAYLOAD)
	$(SG_BOOT) run --main $(DTB)

all: 
	@echo $(YELLOW)"Build STM32F4 standard peripheral driver..."$(RST)
	$(MAKE) all -C STM32
	@echo $(GREEN)"Finish building STM32F4 standard peripheral driver."$(RST)
	@echo ""
	@echo $(YELLOW)"Build CMSIS..."$(RST)
	$(MAKE) all -C CMSIS
	@echo $(GREEN)"Finish building CMSIS."$(RST)
	@echo ""
	@echo $(YELLOW)"Build NRF24..."$(RST)
	$(MAKE) all -C NRF24
	@echo $(GREEN)"Finish building NRF24."$(RST)
	@echo ""
	@echo $(YELLOW)"Build ETH..."$(RST)
	$(MAKE) all -C ETH
	@echo $(GREEN)"Finish building ETH."$(RST)
	@echo ""

clean:
	$(MAKE) clean -e -C CMSIS
	$(MAKE) clean -e -C STM32
	$(MAKE) clean -e -C NRF24
	$(MAKE) clean -e -C ETH
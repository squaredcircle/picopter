picopter: 
	mkdir -p base/obj
	mkdir -p base/bin
	mkdir -p modules/obj
	mkdir -p modules/bin
	mkdir -p apps/obj
	mkdir -p apps/bin
	mkdir -p www-waypoints/obj
	mkdir -p www-waypoints/bin
	$(MAKE) -C base
	$(MAKE) -C modules
	$(MAKE) -C apps
	$(MAKE) -C www-waypoints



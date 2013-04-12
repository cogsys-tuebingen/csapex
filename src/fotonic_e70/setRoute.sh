#!/bin/ash
SETUP_FILE="/fotonic/svc/etc/tof_eth.conf"

if [ -r ${SETUP_FILE} ]
then    # Parse params from setup file
        echo "Using ethernet setup in ${SETUP_FILE}"

        # DHCP params
        USE_DHCP=`grep DHCP ${SETUP_FILE} | cut -d'=' -f2 | sed -e 's/^[ \t]*//'`

        # Static IP params
        GATEWAY=`grep GATEWAY ${SETUP_FILE} | cut -d'=' -f2 | sed -e 's/^[ \t]*//'`

	if [ $USE_DHCP -eq 0 ]; then
		CHECK=`route | grep default | wc -l`
		if [ ! $CHECK -eq 0 ]; then
			echo "resetting route"
			route del default
		fi

		echo "setting route"
		route add default gw $GATEWAY
	fi
fi

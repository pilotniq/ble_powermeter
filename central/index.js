var noble = require('noble-mac');

// var serviceUUIDs = ["<service UUID 1>", ...]; // default: [] => all

var connecting = false;

function startScanning()
{
    console.log( "Searching for Power Meter..." );
    noble.startScanning( ["8a1700018454bea70f4b8876450db486"], true ); // /* serviceUUIDs, allowDuplicates[, callback(error)] */); // particular UUID's
}

//noble.on('scanStart', function() {
	// console.log('on -> scanStart');
//    });

//noble.on('scanStop', function() {
//	console.log('on -> scanStop');
//    });
noble.on('warning', function(message) { console.log( "warning: " + message );} );
noble.on('discover', function( peripheral ) {
	console.log('Found one: ' + peripheral);

	var advertisement = peripheral.advertisement;
	var serviceData = advertisement.serviceData;
	var serviceUuids = advertisement.serviceUuids;

	if (serviceData) {
	    console.log('  Service Data      = ' + JSON.stringify(serviceData, null, 2));
	}

	if (serviceUuids) {
	    console.log('  Service UUIDs     = ' + serviceUuids);
	}

	if( !connecting )
	{
	  connecting = true;
	
	  console.log( "Connecting to Power Meter" );
	peripheral.connect(function(error) {
		console.log('connected to Power Meter: ' + peripheral.uuid);
    
		peripheral.discoverServices(['8a1700018454bea70f4b8876450db486'] , function(error, services) {
			var service = services[0];
			console.log('service: ' + service + ', discovering characteristics' );

			service.discoverCharacteristics(['8a1700028454bea70f4b8876450db486', '8a1700048454bea70f4b8876450db486'], 
							 function(error, characteristics) {
				console.log('discovered the following characteristics:');
				for (var i in characteristics) {
				    if( characteristics[i].uuid == '8a1700028454bea70f4b8876450db486' )
				    {
					console.log( "Found power characteristic, monitoring." );
					// Power (in W)
					characteristics[i].on('data', function(data, isNotification) {
						console.log('power is now: ', data.readUInt16LE(0) + ' W');
					    });
					characteristics[i].subscribe(function(error) {
						console.log('power notification on'); });
				    }
				    else if( characteristics[i].uuid == '8a1700048454bea70f4b8876450db486' )
				    {
					console.log( "Found energy characteristic, monitoring." );
					// Energy (in Wh)
					characteristics[i].on('data', function(data, isNotification) {
						console.log('energy is now: ', data.readUInt32LE(0) + ' Wh');
					    });
					characteristics[i].subscribe(function(error) {
						console.log('energy notification on'); });
				    }
				    else
					console.log('  ' + i + ' uuid: ' + characteristics[i].uuid);
				} // end of for characteristic loop
				// peripheral.disconnect(function(error) { console.log('disconnected from peripheral: ' + peripheral.uuid); });
							}); // end of discovercharacteristics
		    }); // end of peripheral.discoverServices
	    }); // end of peripheral.connect
	// peripheral.once('servicesDiscover', function(services) { console.log( "peripheral " + peripheral + " has services " + services ); } );
	// peripheral.discoverServices();
	} // end of if( connecting )
    } );

noble.on('stateChange', function(state) { if( state === 'poweredOn') startScanning(state); });


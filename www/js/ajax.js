function ajaxSend(action,data) {
	var newdata = [];
	
	if (typeof data != 'undefined') {
		$.each(data, function(index, val) {
			lat = val.getLatLng().lat;
			lng = val.getLatLng().lng;
			newdata.push([lat,lng]);
		});
	}
	
	$.ajax({
		type: "POST",
		url: "ajax-thrift.php",
		data: {
			'action': action,
			'data'   : newdata
		},
		success: function(response) {
			$('#response').html(response);
		}
	});	
}

function allStop() {
	ajaxSend('allStop');
}

function beginAuto() {
	if (!canEdit && bounds.length == 2) {
		ajaxSend('updateWaypoints', bounds);
		setTimeout(function() {
			ajaxSend('beginAuto');
		},500);
	}
}

function beginManual() {
	if (!canEdit && markers.length > 0) {
		ajaxSend('updateWaypoints', markers);
		setTimeout(function() {
			ajaxSend('beginManual');
		},500);
	}
}

function beginUserTracking() {
	if ((navigator.geolocation)) {
		ajaxSend('beginUserTracking');
	}
}

(function worker() {
	if ( typeof userMarker !== 'undefined' ) {
		lat = userMarker.getLatLng().lat;
		lon = userMarker.getLatLng().lng;
		
		data = {
			'action': 'requestAll',
			'lat': lat,
			'lon': lon
		}
	} else {
		data = {
			'action': 'requestAll'
		}
	}
	
	$.ajax({
		type: "POST",
		dataType: "json",
		url:'ajax-thrift.php',
		data: data,
		success: function(data) {
			$("#status").html(data.status);
			
			$("#bearing").html("Facing " + Math.round(data.bearing) + "&deg;");
			
			var latlng = L.latLng(data.lat, data.lon);
			copterMarker.setLatLng(latlng).update();
			
			if (pathEnabled) updatePath(latlng);
			
		},
		error: function() {
			$("#status").html("ERROR: No connection to flight control program.");
		}
	});
	
	if (navigator.geolocation) navigator.geolocation.getCurrentPosition(updateUserPosition);
	
	setTimeout(worker, 2000);
})();

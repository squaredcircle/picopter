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
	if (!canEdit) ajaxSend('beginAuto');
}

function beginManual() {
	if (!canEdit) ajaxSend('beginManual');
}

(function worker() {
	$.ajax({
		type: "POST",
		url:'ajax-thrift.php',
		data: {
			'action': 'requestCoords',
		},
		success: function(data) {
			var arr = data.split(',');
			
			var latlng = L.latLng(parseFloat(arr[0]),parseFloat(arr[1]));

			//copterMarker.setLatLng(latlng).update();
		}
	});
	$.ajax({
		type: "POST",
		url:'ajax-thrift.php',
		data: {
			'action': 'requestStatus',
		},
		success: function(data) {
			$("#status").html(data);
		},
		error: function() {
			$("#status").html("ERROR: No connection to flight control program.");
		}
	});
	setTimeout(worker, 5000);
})();

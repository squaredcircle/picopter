<?php
	$b = Array(false => 'false', true => 'true');

//	print "Received '" . $_POST['action'] . "'<p>";
	
	if (isset($_POST["action"])) {
		switch ($_POST['action']) {
			case "requestCoords":
				$ans = $client->requestCoords();
				print $ans->lat . "," . $ans->lon;
				break;
				
			case "requestStatus":
				$ans = $client->requestStatus();
				print $ans . "\n";
				break;
				
			case "allStop":
				$ans = $client->allStop();
				print "allStop " . $b[$ans];
				break;
				
			case "updateWaypoints":
				if (isset($_POST["data"])) {
					$waypoints = array();
					
					foreach ( $_POST["data"] as $i) {
						$wp = new \picopter\coordDeg();
						$wp->lat = $i[0];
						$wp->lon = $i[1];
						array_push($waypoints, $wp);
					}
					
					$ans = $client->updateWaypoints($waypoints);
					print count($waypoints) . " waypoints added.\n";
				} else {
					print "updateWaypoint failed.\n";
				}
				break;
				
			case "resetWaypoints":
				$ans = $client->resetWaypoints();
				print "resetWaypoints " . $b[$ans] . "\n";
				break;
			
			case "beginManual":
				$ans = $client->beginWaypointsThread();
				print "beginWaypointTraversal " . $b[$ans] . "\n";
				break;
				
			case "beginAuto":
				$ans = $client->beginWaypointTraversal();
				print "beginWaypointTraversal " . $b[$ans] . "\n";
				break;
				
			case "requestNextWaypoint":
				$ans = $client->requestNextWaypoint();
				print $ans-lat . ", " . $ans->lon;
				break;

			default:
				echo "Invalid request: '" . $_POST['action'] . "'";
		}
	} else {
		echo "No request specified";
	}
?>

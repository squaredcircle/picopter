<?php
	$b = Array(false => 'false', true => 'true');

//	print "Received '" . $_POST['action'] . "'<p>";
	
	if (isset($_POST["action"])) {
		switch ($_POST['action']) {
			case "requestCoords":
				$ans = $client->requestCoords();
				print $ans->lat . "," . $ans->lon;
				break;
			case "allStop":
				$ans = $client->allStop();
				print "allStop " . $b[$ans];
				break;
			case "requestStatus":
				$ans = $client->requestStatus();
				print $ans . "\n";
				break;
			case "beginWaypointTraversal":
				$ans = $client->beginWaypointTraversal();
				print "beginWaypointTraversal " . $b[$ans] . "\n";
				break;
			case "requestNextWaypoint":
				$ans = $client->requestNextWaypoint();
				print $ans-lat . ", " . $ans->lon;
				break;
			case "addWaypoint":
				if (isset($_POST["coords"])) {
					$coord = sscanf($_POST["coords"], '(%f, %f)', $lat, $lon);
					$wp = new \picopter\coordDeg();
					$wp->lat = $lat;
					$wp->lon = $lon;
					$ans = $client->addWaypoint($wp);
					print "addWaypoint (" . $lat . ", " . $lon .") " . $b[$ans] . "\n";
				} else {
					print "addWaypoint failed.";
				}
				break;
			case "updateWaypoint":
				if (isset($_POST["coords"]) && isset($_POST["no"])) {
					$coord = sscanf($_POST["coords"], '(%f, %f)', $lat, $lon);
					$wp = new \picopter\coordDeg();
					$wp->lat = $lat;
					$wp->lon = $lon;
					$ans = $client->updateWaypoint($wp,$_POST["no"]);
					print "updateWaypoint " . $_POST["no"] . " (" . $lat . ", " . $lon .") " . $b[$ans] . "\n";
				} else {
					print "updateWaypoint failed.";
				}
				break;
		
			case "removeWaypoints":
				$ans = $client->removeWaypoints();
				print "removeWaypoints " . $b[$ans] . "\n";
				break;
			case "resetWaypoints":
				$ans = $client->resetWaypoints();
				print "resetWaypoints " . $b[$ans] . "\n";
				break;
			default:
				echo "Invalid request: '" . $_POST['action'] . "'";
		}
	} else {
		echo "No request specified";
	}
?>

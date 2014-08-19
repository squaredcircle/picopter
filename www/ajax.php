<?php
	$f = file_get_contents('/home/pi/current_gps.txt');
	$coords = explode(',',$f);
	
	echo "
		<p><b>Latitude:</b> ".$coords[0]."&deg; N</p>
		<p><b>Longtitude:</b> ".$coords[1]."&deg; E</p>";
?>

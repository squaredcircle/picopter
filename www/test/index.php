<html>
<body>

<p><button class="things" value="first">First Button</button>
<p><button class="things" value="second">Second Button</button>
<p><button class="things" value="third">Third Button</button>
<p><button class="things" value="fourth">Fourth Button</button>
<p><div id="response"></div>

<script src="../js/jquery-2.1.1.min.js"></script>
<script>
	$(".things").click(function(){
		var clickBtnValue = $(this).attr("value");
		$.ajax({
			type: "POST",
			url: "ajax.php",
			data: {
				'action': clickBtnValue,
				'more' : 'heythere'
			},
			success: function(data) {
				$('#response').html(data);
			}
		});
	});
</script>

</body>
</html>

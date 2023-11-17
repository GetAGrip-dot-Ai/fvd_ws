// static/main.js
$(document).ready(function () {
    var selectedPoint = { x: 0, y: 0 };

    $('#image').click(function (event) {
        var offset = $(this).offset();
        var clickX = event.pageX - offset.left;
        var clickY = event.pageY - offset.top;

        selectedPoint.x = clickX;
        selectedPoint.y = clickY;

        $('#selected-point').text(`Selected Point: (${clickX}, ${clickY})`);
    });

    $('#send-button').click(function () {
        // Send selectedPoint as JSON with the correct content type
        $.ajax({
            type: 'POST',
            url: '/send_to_ros',
            data: JSON.stringify(selectedPoint),
            contentType: 'application/json',  // Set the content type to JSON
            success: function (response) {
                console.log(response);
            },
        });
    });
    $('#reset-button').click(function () {
        // Send selectedPoint as JSON with the correct content type
        selectedPoint.x = -1;
        selectedPoint.y = -1;
        $.ajax({
            type: 'POST',
            url: '/send_to_ros',
            data: JSON.stringify(selectedPoint),
            contentType: 'application/json',  // Set the content type to JSON
            success: function (response) {
                console.log(response);
            },
        });
    });
});

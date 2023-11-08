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
        // Send selectedPoint to ROS using an AJAX request or WebSocket
        // Example:
        $.post('/send_to_ros', { x: selectedPoint.x, y: selectedPoint.y }, function (response) {
            console.log(response);
        });
    });
});

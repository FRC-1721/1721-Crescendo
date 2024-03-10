var fieldRelative = false;

$(function () {
    // sets a function that will be called when the websocket connects/disconnects
    NetworkTables.addWsConnectionListener(onNetworkTablesConnection, true);

    // sets a function that will be called when the robot connects/disconnects
    NetworkTables.addRobotConnectionListener(onRobotConnection, true);

    // sets a function that will be called when any NetworkTables key/value changes
    NetworkTables.addGlobalListener(onValueChanged, true);

    /* // Doesn't work, using img, src=<ip>
    loadCameraOnConnect({
        container: "#cam1", // id of camera div
        proto: null, // url scheme
        host: "10.17.21.11", // ip
        port: 5800,
        image_url: "/",
        data_url: "/",
        wait_img: require("./assets/no_signal.png"),
        error_img: require("./assets/error.png"),
        attrs: {
            width: 320,
            height: 240,
        },
    });
    */
});

function onRobotConnection(connected) {
    $("#robotstate").text(connected ? "Connected" : "Disconnected");
    $("#robotAddress").text(
        connected ? NetworkTables.getRobotAddress() : "Disconnected"
    );
}

function onNetworkTablesConnection(connected) {
    if (connected) {
        $("#connectstate").text("Connected");

        // clear the table
        $("#nt tbody > tr").remove();
    } else {
        $("#connectstate").text("Disconnected");
    }
}

function ntToggle(event) {
    $(event.parentElement).toggleClass("disabled");
}

$("#fieldrelative").on("click", function () {
    NetworkTables.putValue(
        "/SmartDashboard/FieldCentric/.controllable",
        !$(this).is(".active")
    );
});

function putNT(key, value) {
    function put(items, path, data) {
        var [x, ...xs] = items;
        if (xs.length) {
            // if the item has children then it must be a folder
            // CREATE folder IF NOT EXISTS
            if (
                $("#" + NetworkTables.keySelector(path + "/" + x)).length == 0
            ) {
                var div = $(" <div /> ", {
                    id: NetworkTables.keyToId(path + "/" + x),
                    class: "nt-div disabled",
                });
                // folder key label
                $("<div />", {
                    id: NetworkTables.keyToId(path + "/" + x + "_title"),
                    class: "nt-title",
                    text: x,
                    onClick: "ntToggle(this)",
                }).appendTo(div);
                // folder children area
                $("<div />", {
                    id: NetworkTables.keyToId(path + "/" + x + "/"),
                    class: "nt-div-container",
                }).appendTo(div);
                // add folder to parent folder
                $("#" + NetworkTables.keySelector(path + "/")).append(div);
            }
            put(xs, path + "/" + x, data); // make current folder parent folder and call function again
        } else {
            // if the item doesn't have children then it must not be a folder
            var item = $(" <div /> ", {
                id: NetworkTables.keyToId(path + "/" + x),
                class: "nt-item",
            });
            // item key
            $(" <div /> ", {
                class: "nt-key",
                text: x,
            }).appendTo(item);
            // item value
            $(" <div /> ", {
                class: "nt-value",
                id: NetworkTables.keyToId(path + "/" + x),
                text: data,
            }).appendTo(item);
            // add item to parent folder
            $("#" + NetworkTables.keySelector(path + "/")).append(item);
        }
    }
    var [_, ...items] = key.split("/");
    // start function with initial path blank
    put(items, "", value);
}

function onValueChanged(key, value, isNew) {
    // key thing here: we're using the various NetworkTable keys as
    // the id of the elements that we're appending, for simplicity. However,
    // the key names aren't always valid HTML identifiers, so we use
    // the NetworkTables.keyToId() function to convert them appropriately

    if (isNew) {
        putNT(key, value);
    } else {
        // similarly, use keySelector to convert the key to a valid jQuery
        // selector. This should work for class names also, not just for ids
        $("#" + NetworkTables.keySelector(key) + " .nt-value").text(value);
    }

    if (key.includes("/SmartDashboard/Audio")) {
        countDownAlerts(key, value);
    }

    if (key.includes("/SmartDashboard/Swerve/")) {
        if (key.includes("desired")) {
            wheel = key.split("/").at(-2);
            $(".swerve-desired ." + wheel + "d").css(
                "transform",
                "rotate(" + value + "deg)"
            );
        } else {
            wheel = key.split("/").at(-2);
            $(".swerve ." + wheel).css("transform", "rotate(" + value + "deg)");
        }
    }

    if (key.includes("/SmartDashboard/FieldCentric/.controllable")) {
        $("#fieldrelative").toggleClass("active", value);
        $("#fieldrelative .indicator").text(value ? "⏽" : "⭘");
    }
}

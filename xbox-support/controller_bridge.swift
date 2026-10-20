// Read-only macOS GameController bridge. No robot/network/control commands.
import Foundation
import AppKit
import GameController

let app = NSApplication.shared
app.setActivationPolicy(.accessory)
GCController.shouldMonitorBackgroundEvents = true

func emitController() {
    let controllers = GCController.controllers().filter { $0.extendedGamepad != nil }
    var result: [String: Any] = [
        "version": 2,
        "connected": false,
        "name": controllers.count > 1
            ? "Multiple gamepads connected. Disconnect the extra controller."
            : "No controller detected. Connect or pair your Xbox controller."
    ]
    if controllers.count == 1, let controller = controllers.first,
       let pad = controller.extendedGamepad {
        var buttons: [String] = []
        if pad.buttonMenu.isPressed { buttons.append("menu") }
        if pad.buttonOptions?.isPressed == true { buttons.append("view") }
        if pad.buttonB.isPressed { buttons.append("b") }
        if pad.buttonX.isPressed { buttons.append("x") }
        if pad.buttonA.isPressed { buttons.append("a") }
        if pad.buttonY.isPressed { buttons.append("y") }
        if pad.leftShoulder.isPressed { buttons.append("lb") }
        if pad.rightShoulder.isPressed { buttons.append("rb") }
        if pad.leftThumbstickButton?.isPressed == true { buttons.append("l3") }
        if pad.rightThumbstickButton?.isPressed == true { buttons.append("r3") }
        result = [
            "version": 2,
            "connected": true,
            "name": "\(controller.productCategory) · \(controller.vendorName ?? "Controller")",
            "axes": [
                pad.leftThumbstick.xAxis.value, pad.leftThumbstick.yAxis.value,
                pad.rightThumbstick.xAxis.value, pad.rightThumbstick.yAxis.value,
                pad.leftTrigger.value, pad.rightTrigger.value
            ],
            "buttons": buttons
        ]
    }
    if let data = try? JSONSerialization.data(withJSONObject: result),
       let text = String(data: data, encoding: .utf8) {
        print(text)
        fflush(stdout)
    }
}

let timer = Timer(timeInterval: 1.0 / 60.0, repeats: true) { _ in emitController() }
RunLoop.main.add(timer, forMode: .common)
RunLoop.main.run()

//
//  ViewController.swift
//  iOS_ros_stream
//
//  Created by scope on 6/12/18.
//  Copyright © 2018 occamlab. All rights reserved.
//

import UIKit
import ARKit
import Foundation

class ViewController: UIViewController {
    
    @IBOutlet var logView: UITextView!
    
    var broadcastConnection: UDPBroadcastConnection!
    
    var configuration: ARWorldTrackingConfiguration!
    
    var timer = Timer()
    
    @IBOutlet var sceneView: ARSCNView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        startSession()

        broadcastConnection = UDPBroadcastConnection(port: Config.Ports.broadcast) { [unowned self] (ipAddress: String, port: Int, response: [UInt8]) -> Void in
            let log = "Received from \(ipAddress):\(port):\n\n\(response)"
            self.logView.text = log
        }
    }

    @IBAction func reload(_ sender: Any) {
        self.logView.text = "I don't know."
        scheduledTimerToTransmitData()
    }
    
    func scheduledTimerToTransmitData() {
        timer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitData), userInfo: nil, repeats: true)
    }
    
    @objc func transmitData() {
        broadcastConnection.sendBroadcast(getCameraCoordinates(sceneView: sceneView))
    }
    
    func startSession() {
        let configuration = ARWorldTrackingConfiguration()
        sceneView.session.run(configuration)
    }
    
    func getCameraCoordinates(sceneView: ARSCNView) -> String {
        let camera = sceneView.session.currentFrame?.camera
        let cameraTransform = camera?.transform
        let eulers = camera?.eulerAngles
        let scene = SCNMatrix4(cameraTransform!)
        
        let roll = eulers?.z
        let yaw = eulers?.y
        let pitch = eulers?.x
        
        let iosTime = Date()
        let relativeTime = iosTime.timeIntervalSince1970
        
        let fullMatrix = String(format: "%f,%f,%f,%f,%f,%f,%f", scene.m41, scene.m42, scene.m43, relativeTime, roll!, pitch!, yaw!)

        return fullMatrix
    }
    
    
}

extension ViewController: ARSCNViewDelegate {
    func session(_ session: ARSession, didFailWithError error: Error) {
        print("Session Failed - probably due to lack of camera access")
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        print("Session interrupted")
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        print("Session resumed")
        sceneView.session.run(session.configuration!,
                              options: [.resetTracking,
                                        .removeExistingAnchors])
    }
    
    
}


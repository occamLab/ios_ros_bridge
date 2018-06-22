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
    
    //MARK: Properties
    
    var broadcastPoseConnection: UDPBroadcastConnection!
    var broadcastImagesConnection: UDPBroadcastConnection!
    var broadcastIntrinsicsConnection: UDPBroadcastConnection!
    
    var configuration: ARWorldTrackingConfiguration!
    
    var poseTimer = Timer()
    var imageTimer = Timer()
    var intrinsicsTimer = Timer()
    
    var imageIndex = 0          // this is the sequence number in the image stream
    
    @IBOutlet weak var ipAddressText: UITextField!
    
    @IBOutlet var sceneView: ARSCNView!
    
    /// Initializes an ARSession when the app first loads
    override func viewDidLoad() {
        super.viewDidLoad()
        startSession()
    }
    
    /// Data is sent to a new ip address anytime a button is pressed
    @IBAction func reload(_ sender: Any) {
        let INADDR_BROADCAST = in_addr(s_addr: inet_addr(ipAddressText.text))
        
        broadcastPoseConnection = UDPBroadcastConnection(port: Config.Ports.broadcast, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
        
        broadcastImagesConnection = UDPBroadcastConnection(port: Config.Ports.broadcastImages, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
        
        broadcastIntrinsicsConnection = UDPBroadcastConnection(port: Config.Ports.broadcastCameraIntrinsics, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
        
        scheduledTimerToTransmitData()
    }
    
    /// Initializes timers to send data at regular intervals
    func scheduledTimerToTransmitData() {
        poseTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitPoseData), userInfo: nil, repeats: true)
    
        imageTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitImages), userInfo: nil, repeats: true)
        
        intrinsicsTimer = Timer.scheduledTimer(timeInterval: 2, target: self, selector: #selector(self.transmitIntrinsics), userInfo: nil, repeats: true)
    }
    
    /// Sends the pose data to ROS
    @objc func transmitPoseData() {
        broadcastPoseConnection.sendBroadcast(getCameraCoordinates(sceneView: sceneView))
    }
    
    /// Sends the camera intrinsics to ROS
    @objc func transmitIntrinsics() {
        broadcastIntrinsicsConnection.sendBroadcast(getCameraIntrinsics(sceneView: sceneView))
    }
    
    /// Sends the camera frames to ROS
    @objc func transmitImages() {
        let MTU = 1450
        let (imageData, stampedTime) = getVideoFrames(sceneView: sceneView)
        let frameTime = String(stampedTime).data(using: .utf8)
        var bytesSent = 0           // Keeps track of how much of the image has been sent
        var packetIndex = 0         // Packet number - so ROS can recompile the image in order
        
        while bytesSent < imageData.count {
            // Construct the range for the packet
            let range = Range(bytesSent..<min(bytesSent + MTU, imageData.count))
            var udpPacketPayload = imageData.subdata(in: range)
            udpPacketPayload.insert(UInt8(imageIndex % 256), at: 0)     // TODO: remove 256 as magic number
            udpPacketPayload.insert(UInt8(packetIndex), at: 1)
            
            if bytesSent == 0 {
                let numPackets = (Float(imageData.count) / Float(MTU)).rounded(.up)
                udpPacketPayload.insert(UInt8(numPackets), at: 2)
                udpPacketPayload.insert(UInt8(frameTime!.count), at: 3)
                udpPacketPayload.insert(contentsOf: frameTime!, at: 4)
            }
            broadcastImagesConnection.sendBroadcast(udpPacketPayload)
            bytesSent += range.count
            packetIndex += 1
        }
        imageIndex += 1
    }
    

    /// Initialize the ARSession
    func startSession() {
        let configuration = ARWorldTrackingConfiguration()
        /*
         if #available(iOS 11.3, *) {
         configuration.planeDetection = [.horizontal, .vertical]
         }
         else {
         configuration.planeDetection = [.horizontal]
         }
         */
        sceneView.session.run(configuration)
    }
    
    /// Get data (transformation matrix, time) and send to ROS.
    func getCameraCoordinates(sceneView: ARSCNView) -> String {
        let camera = sceneView.session.currentFrame?.camera
        let cameraTransform = camera?.transform
        let scene = SCNMatrix4(cameraTransform!)
        
        let iosTime = Date()
        let relativeTime = iosTime.timeIntervalSince1970
        
        let fullMatrix = String(format: "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", scene.m11, scene.m12, scene.m13, scene.m14, scene.m21, scene.m22, scene.m23, scene.m24, scene.m31, scene.m32, scene.m33, scene.m34, scene.m41, scene.m42, scene.m43, scene.m44, relativeTime)
        
        return fullMatrix
    }
    
    /// Get video frames to send to ROS.
    func getVideoFrames(sceneView: ARSCNView) -> (Data, Double) {
        let cameraFrame = sceneView.session.currentFrame
        let stampedTime = cameraFrame?.timestamp
        
        // Convert ARFrame to a jpeg
        let pixelBuffer = cameraFrame?.capturedImage
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer!)
        let context = CIContext(options: nil)
        let cgImage = context.createCGImage(ciImage, from: ciImage.extent)
        let uiImage = UIImage(cgImage: cgImage!)
        let jpeg = UIImageJPEGRepresentation(uiImage, 0)
        
        return (jpeg!, stampedTime!)
    }
    
    /// Get the camera intrinsics to send to ROS
    func getCameraIntrinsics(sceneView: ARSCNView) -> String {
        let camera = sceneView.session.currentFrame?.camera
        let intrinsics = camera?.intrinsics
        let columns = intrinsics?.columns
        let res = camera?.imageResolution
        let width = res?.width
        let height = res?.height
        
        return String(format: "%f,%f,%f,%f,%f,%f,%f", columns!.0.x, columns!.1.y, columns!.2.x, columns!.2.y, columns!.2.z, width!, height!)
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


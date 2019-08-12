
"use strict";

let MissionHpUpdateYawRate = require('./MissionHpUpdateYawRate.js')
let MFIOConfig = require('./MFIOConfig.js')
let SDKControlAuthority = require('./SDKControlAuthority.js')
let MissionHpAction = require('./MissionHpAction.js')
let SendMobileData = require('./SendMobileData.js')
let DroneArmControl = require('./DroneArmControl.js')
let MFIOSetValue = require('./MFIOSetValue.js')
let MissionWpAction = require('./MissionWpAction.js')
let QueryDroneVersion = require('./QueryDroneVersion.js')
let SendPayloadData = require('./SendPayloadData.js')
let Stereo240pSubscription = require('./Stereo240pSubscription.js')
let MissionWpSetSpeed = require('./MissionWpSetSpeed.js')
let Activation = require('./Activation.js')
let MissionWpUpload = require('./MissionWpUpload.js')
let MissionHpGetInfo = require('./MissionHpGetInfo.js')
let MissionHpResetYaw = require('./MissionHpResetYaw.js')
let SetHardSync = require('./SetHardSync.js')
let StereoVGASubscription = require('./StereoVGASubscription.js')
let DroneTaskControl = require('./DroneTaskControl.js')
let MissionWpGetSpeed = require('./MissionWpGetSpeed.js')
let SetLocalPosRef = require('./SetLocalPosRef.js')
let SetupCameraStream = require('./SetupCameraStream.js')
let MissionStatus = require('./MissionStatus.js')
let MissionHpUpload = require('./MissionHpUpload.js')
let MissionHpUpdateRadius = require('./MissionHpUpdateRadius.js')
let MissionWpGetInfo = require('./MissionWpGetInfo.js')
let StereoDepthSubscription = require('./StereoDepthSubscription.js')
let CameraAction = require('./CameraAction.js')

module.exports = {
  MissionHpUpdateYawRate: MissionHpUpdateYawRate,
  MFIOConfig: MFIOConfig,
  SDKControlAuthority: SDKControlAuthority,
  MissionHpAction: MissionHpAction,
  SendMobileData: SendMobileData,
  DroneArmControl: DroneArmControl,
  MFIOSetValue: MFIOSetValue,
  MissionWpAction: MissionWpAction,
  QueryDroneVersion: QueryDroneVersion,
  SendPayloadData: SendPayloadData,
  Stereo240pSubscription: Stereo240pSubscription,
  MissionWpSetSpeed: MissionWpSetSpeed,
  Activation: Activation,
  MissionWpUpload: MissionWpUpload,
  MissionHpGetInfo: MissionHpGetInfo,
  MissionHpResetYaw: MissionHpResetYaw,
  SetHardSync: SetHardSync,
  StereoVGASubscription: StereoVGASubscription,
  DroneTaskControl: DroneTaskControl,
  MissionWpGetSpeed: MissionWpGetSpeed,
  SetLocalPosRef: SetLocalPosRef,
  SetupCameraStream: SetupCameraStream,
  MissionStatus: MissionStatus,
  MissionHpUpload: MissionHpUpload,
  MissionHpUpdateRadius: MissionHpUpdateRadius,
  MissionWpGetInfo: MissionWpGetInfo,
  StereoDepthSubscription: StereoDepthSubscription,
  CameraAction: CameraAction,
};

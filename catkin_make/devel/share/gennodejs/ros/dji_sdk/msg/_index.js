
"use strict";

let MissionWaypoint = require('./MissionWaypoint.js');
let Gimbal = require('./Gimbal.js');
let GPSUTC = require('./GPSUTC.js');
let MissionWaypointAction = require('./MissionWaypointAction.js');
let MissionWaypointTask = require('./MissionWaypointTask.js');
let WaypointList = require('./WaypointList.js');
let FCTimeInUTC = require('./FCTimeInUTC.js');
let FlightAnomaly = require('./FlightAnomaly.js');
let PayloadData = require('./PayloadData.js');
let MobileData = require('./MobileData.js');
let VOPosition = require('./VOPosition.js');
let MissionHotpointTask = require('./MissionHotpointTask.js');
let Waypoint = require('./Waypoint.js');

module.exports = {
  MissionWaypoint: MissionWaypoint,
  Gimbal: Gimbal,
  GPSUTC: GPSUTC,
  MissionWaypointAction: MissionWaypointAction,
  MissionWaypointTask: MissionWaypointTask,
  WaypointList: WaypointList,
  FCTimeInUTC: FCTimeInUTC,
  FlightAnomaly: FlightAnomaly,
  PayloadData: PayloadData,
  MobileData: MobileData,
  VOPosition: VOPosition,
  MissionHotpointTask: MissionHotpointTask,
  Waypoint: Waypoint,
};

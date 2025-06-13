export const ws_Methods = {
    STATUS: 'status',
    ERROR: 'error',
    IDENTIFY: 'identify',
    LAUNCH_MISSION: 'launchMission',
    STOP_MISSION: 'stopMission',
    MISSION_AVAILABILITY: 'missionAvailability',
    START_ROS: 'start_ros',
    TIMER_UPDATE: 'timerUpdate',
    GET_BATTERY: 'get_battery',
    LASER_SCAN_LOG: 'laser_scan_log',
    FETCH_MISSION: 'fetchOldMission',
    MISSION_STATUS_UPDATE: 'missionStatusUpdate',
    MAP_IMAGE_UPDATE: "map_image_update"

};

export enum EventType {
    MISSION_STATUS = "MISSION_STATUS",
    LOGS = "LOGS",
    BATTERY_LEVEL = "BATTERY_LEVEL",
    OBJECT_DETECTION = "OBJECT_DETECTION",
    CONNECTION_STATUS = "CONNECTION_STATUS",
    IDENTIFICATION = "IDENTIFICATION_STATUS",
    MAP = "MAP"
}

export enum ModeMission {
    SIMULATION = "SIMULATION",
    PHYSICAL_ROBOT = "PHYSICAL_ROBOT"
}
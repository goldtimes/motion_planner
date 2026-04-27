
"use strict";

let DetectedPerson = require('./DetectedPerson.js');
let DetectedPersons = require('./DetectedPersons.js');
let PersonTrajectoryEntry = require('./PersonTrajectoryEntry.js');
let TrackedPersons = require('./TrackedPersons.js');
let PersonTrajectory = require('./PersonTrajectory.js');
let CompositeDetectedPersons = require('./CompositeDetectedPersons.js');
let CompositeDetectedPerson = require('./CompositeDetectedPerson.js');
let ImmDebugInfos = require('./ImmDebugInfos.js');
let TrackedPersons2d = require('./TrackedPersons2d.js');
let ImmDebugInfo = require('./ImmDebugInfo.js');
let TrackedGroups = require('./TrackedGroups.js');
let TrackedPerson2d = require('./TrackedPerson2d.js');
let TrackedPerson = require('./TrackedPerson.js');
let TrackingTimingMetrics = require('./TrackingTimingMetrics.js');
let TrackedGroup = require('./TrackedGroup.js');

module.exports = {
  DetectedPerson: DetectedPerson,
  DetectedPersons: DetectedPersons,
  PersonTrajectoryEntry: PersonTrajectoryEntry,
  TrackedPersons: TrackedPersons,
  PersonTrajectory: PersonTrajectory,
  CompositeDetectedPersons: CompositeDetectedPersons,
  CompositeDetectedPerson: CompositeDetectedPerson,
  ImmDebugInfos: ImmDebugInfos,
  TrackedPersons2d: TrackedPersons2d,
  ImmDebugInfo: ImmDebugInfo,
  TrackedGroups: TrackedGroups,
  TrackedPerson2d: TrackedPerson2d,
  TrackedPerson: TrackedPerson,
  TrackingTimingMetrics: TrackingTimingMetrics,
  TrackedGroup: TrackedGroup,
};

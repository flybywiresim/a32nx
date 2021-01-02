/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Working Title, FlyByWire Simulations
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import { WTDataStore } from "WorkingTitle";
import { FlightPlanManager } from "./FlightPlanManager";

/** A class for syncing a flight plan with the game */
export class FlightPlanAsoboSync {
  static fpChecksum: number = 0;
  static fpListenerInitialized = false;

  public static init() {
    if(!this.fpListenerInitialized){
      RegisterViewListener("JS_LISTENER_FLIGHTPLAN");
      this.fpListenerInitialized = true;
    }
  }

  public static async LoadFromGame(fpln: FlightPlanManager): Promise<void> {
    return new Promise((resolve, reject) => {
      this.init();
      setTimeout(() => {
        Coherent.call("LOAD_CURRENT_GAME_FLIGHT");
        Coherent.call("LOAD_CURRENT_ATC_FLIGHTPLAN");
        setTimeout(() => {
          Coherent.call("GET_FLIGHTPLAN").then(async (data) => {
            console.log("COHERENT GET_FLIGHTPLAN received");
            const isDirectTo = data.isDirectTo;

            // TODO: talk to matt about dirto
            if (!isDirectTo) {
              if (data.waypoints.length === 0) {
                resolve();
                return;
              }

              await fpln._parentInstrument.facilityLoader.getFacilityRaw(data.waypoints[0].icao, 10000);

              // set origin
              await fpln.setOrigin(data.waypoints[0].icao);

              // set dest
              await fpln.setDestination(data.waypoints[data.waypoints.length - 1].icao);

              // set route
              const enrouteStart = (data.departureWaypointsSize == -1) ? 1 : data.departureWaypointsSize
              const enroute = data.waypoints.slice(enrouteStart);
              for (let i = 0; i < enroute.length - 1; i++) {
                const wpt = enroute[i];
                console.log(wpt.icao);
                if (wpt.icao.trim() !== "") {
                  await fpln.addWaypoint(wpt.icao);
                }
              }

              // set departure
              //  rwy index
              await fpln.setOriginRunwayIndex(data.originRunwayIndex);
              await fpln.setDepartureRunwayIndex(data.departureRunwayIndex);
              //  proc index
              await fpln.setDepartureProcIndex(data.departureProcIndex);
              //  enroutetrans index
              await fpln.setDepartureEnRouteTransitionIndex(data.departureEnRouteTransitionIndex);

              // set arrival
              //  arrivalproc index
              await fpln.setArrivalProcIndex(data.arrivalProcIndex);
              //  arrivaltrans index
              await fpln.setArrivalEnRouteTransitionIndex(data.arrivalEnRouteTransitionIndex);

              // set approach
              //  approach index
              await fpln.setApproachIndex(data.approachIndex);
              //  approachtrans index
              await fpln.setApproachTransitionIndex(data.approachTransitionIndex);

              this.fpChecksum = fpln.getCurrentFlightPlan().checksum;
              resolve();
            }
          });
        }, 500);
      }, 200);
    });
  }

  public static async SaveToGame(fpln: FlightPlanManager): Promise<void> {
    return new Promise(async (resolve, reject) => {
      const plan = fpln.getCurrentFlightPlan();
      if (WTDataStore.get('WT_CJ4_FPSYNC', 0) !== 0 && (plan.checksum !== this.fpChecksum)) {

        // await Coherent.call("CREATE_NEW_FLIGHTPLAN");
        await Coherent.call("SET_CURRENT_FLIGHTPLAN_INDEX", 0).catch(console.log);
        await Coherent.call("CLEAR_CURRENT_FLIGHT_PLAN").catch(console.log);

        if (plan.hasOrigin && plan.hasDestination) {
          if (plan.hasOrigin) {
            await Coherent.call("SET_ORIGIN", plan.originAirfield.icao);
          }

          if (plan.hasDestination) {
            await Coherent.call("SET_DESTINATION", plan.destinationAirfield.icao);
          }

          let coIndex = 1;
          for (let i = 0; i < plan.enroute.waypoints.length; i++) {
            const wpt = plan.enroute.waypoints[i];
            if (wpt.icao.trim() !== "") {
              await Coherent.call("ADD_WAYPOINT", wpt.icao, coIndex, false);
              coIndex++;
            }
          }

          await Coherent.call("SET_ACTIVE_WAYPOINT_INDEX", fpln.getActiveWaypointIndex());

          await Coherent.call("SET_ORIGIN_RUNWAY_INDEX", plan.procedureDetails.originRunwayIndex).catch(console.log);
          await Coherent.call("SET_DEPARTURE_RUNWAY_INDEX", plan.procedureDetails.departureRunwayIndex);
          await Coherent.call("SET_DEPARTURE_PROC_INDEX", plan.procedureDetails.departureIndex);
          await Coherent.call("SET_DEPARTURE_ENROUTE_TRANSITION_INDEX", plan.procedureDetails.departureTransitionIndex);

          await Coherent.call("SET_ARRIVAL_RUNWAY_INDEX", plan.procedureDetails.arrivalRunwayIndex);
          await Coherent.call("SET_ARRIVAL_PROC_INDEX", plan.procedureDetails.arrivalIndex);
          await Coherent.call("SET_ARRIVAL_ENROUTE_TRANSITION_INDEX", plan.procedureDetails.arrivalTransitionIndex);

          await Coherent.call("SET_APPROACH_INDEX", plan.procedureDetails.approachIndex).then(() => {
            Coherent.call("SET_APPROACH_TRANSITION_INDEX", plan.procedureDetails.approachTransitionIndex);
          });
        }

        this.fpChecksum = plan.checksum;
        Coherent.call("RECOMPUTE_ACTIVE_WAYPOINT_INDEX");
      }
    });
  }
}
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

/**
 * The details of procedures selected in the flight plan.
 */
export class ProcedureDetails {
    /** The index of the origin runway in the origin runway information. */
    public originRunwayIndex: number = -1;

    /** The index of the departure in the origin airport information. */
    public departureIndex: number = -1;

    /** The index of the departure transition in the origin airport departure information. */
    public departureTransitionIndex: number = -1;

    /** The index of the selected runway in the original airport departure information. */
    public departureRunwayIndex: number = -1;

    /** The index of the arrival in the destination airport information. */
    public arrivalIndex: number = -1;

    /** The index of the arrival transition in the destination airport arrival information. */
    public arrivalTransitionIndex: number = -1;

    /** The index of the selected runway transition at the destination airport arrival information. */
    public arrivalRunwayIndex: number = -1;

    /** The index of the apporach in the destination airport information.*/
    public approachIndex: number = -1;

    /** The index of the approach transition in the destination airport approach information.*/
    public approachTransitionIndex: number = -1;

    /** The index of the destination runway in the destination runway information. */
    public destinationRunwayIndex = -1;

    /** The length from the threshold of the runway extension fix. */
    public destinationRunwayExtension = -1;
}
import { vec3, vec2, box3, triangle3 } from 'mathcat';
import { isValidNodeRef, findLocalNeighbourhood, getPolyWallSegments, moveAlongSurface, findStraightPath, NodeType, INVALID_NODE_REF, getNodeByRef, createSlicedNodePathQuery, initSlicedFindNodePath, updateSlicedFindNodePath, finalizeSlicedFindNodePathPartial, SlicedFindNodePathStatusFlags, raycast, createFindNearestPolyResult, findNearestPoly, StraightPathPointFlags, finalizeSlicedFindNodePath, createGetClosestPointOnPolyResult, getClosestPointOnPoly, BuildContext, markWalkableTriangles, calculateMeshBounds, calculateGridSize, createHeightfield, rasterizeTriangles, filterLowHangingWalkableObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, buildCompactHeightfield, erodeWalkableArea, buildDistanceField, buildRegions, buildContours, ContourBuildFlags, buildPolyMesh, WALKABLE_AREA, buildPolyMeshDetail, createNavMesh, polyMeshToTilePolys, polyMeshDetailToTileDetailMesh, buildTile, addTile, getNodeRefIndex, getNodeByTileAndPoly } from 'navcat';

const MAX_LOCAL_SEGS = 8;
const MAX_LOCAL_POLYS = 16;
/**
 * Creates a new local boundary instance.
 */
const create$2 = () => ({
    center: [Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE],
    segments: [],
    polys: [],
});
/**
 * Resets the boundary data.
 */
const resetLocalBoundary = (boundary) => {
    vec3.set(boundary.center, Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE);
    boundary.segments.length = 0;
    boundary.polys.length = 0;
};
/**
 * Calculates distance squared from point to line segment in 2D (XZ plane).
 */
const distancePtSegSqr2d = (pt, segStart, segEnd) => {
    const pqx = segEnd[0] - segStart[0];
    const pqz = segEnd[2] - segStart[2];
    const dx = pt[0] - segStart[0];
    const dz = pt[2] - segStart[2];
    const d = pqx * pqx + pqz * pqz;
    let t = pqx * dx + pqz * dz;
    if (d > 0)
        t /= d;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;
    const nearestX = segStart[0] + t * pqx;
    const nearestZ = segStart[2] + t * pqz;
    const distX = pt[0] - nearestX;
    const distZ = pt[2] - nearestZ;
    return distX * distX + distZ * distZ;
};
/**
 * Adds a wall segment to the boundary, sorted by distance.
 */
const addSegmentToBoundary = (boundary, dist, s) => {
    // find insertion point based on distance
    let insertIdx = 0;
    for (let i = 0; i < boundary.segments.length; i++) {
        if (dist <= boundary.segments[i].d) {
            insertIdx = i;
            break;
        }
        insertIdx = i + 1;
    }
    // don't exceed max segments
    if (boundary.segments.length >= MAX_LOCAL_SEGS) {
        // if we're trying to insert past the end, skip
        if (insertIdx >= MAX_LOCAL_SEGS)
            return;
        // remove last segment to make room
        boundary.segments.pop();
    }
    // create new segment
    const segment = {
        d: dist,
        s: [s[0], s[1], s[2], s[3], s[4], s[5]],
    };
    // insert at the correct position
    boundary.segments.splice(insertIdx, 0, segment);
};
/**
 * Updates the local boundary data around the given position.
 * @param boundary The local boundary to update
 * @param nodeRef Current polygon reference
 * @param position Current position
 * @param collisionQueryRange Query range for finding nearby walls
 * @param navMesh Navigation mesh
 * @param filter Query filter
 */
const updateLocalBoundary = (boundary, nodeRef, position, collisionQueryRange, navMesh, filter) => {
    if (!isValidNodeRef(navMesh, nodeRef)) {
        resetLocalBoundary(boundary);
        return;
    }
    vec3.copy(boundary.center, position);
    // first query non-overlapping polygons
    const neighbourhoodResult = findLocalNeighbourhood(navMesh, nodeRef, position, collisionQueryRange, filter);
    if (!neighbourhoodResult.success) {
        boundary.segments.length = 0;
        boundary.polys.length = 0;
        return;
    }
    // store found polygons (limit to max)
    boundary.polys = neighbourhoodResult.nodeRefs.slice(0, MAX_LOCAL_POLYS);
    // clear existing segments
    boundary.segments.length = 0;
    // store all polygon wall segments
    const collisionQueryRangeSqr = collisionQueryRange * collisionQueryRange;
    for (const polyRef of boundary.polys) {
        const wallSegmentsResult = getPolyWallSegments(navMesh, polyRef, filter, false);
        if (!wallSegmentsResult.success)
            continue;
        const segmentCount = wallSegmentsResult.segmentVerts.length / 6;
        for (let k = 0; k < segmentCount; ++k) {
            const segStart = k * 6;
            const s = wallSegmentsResult.segmentVerts.slice(segStart, segStart + 6);
            // skip distant segments
            const segmentStart = [s[0], s[1], s[2]];
            const segmentEnd = [s[3], s[4], s[5]];
            const distSqr = distancePtSegSqr2d(position, segmentStart, segmentEnd);
            if (distSqr > collisionQueryRangeSqr) {
                continue;
            }
            addSegmentToBoundary(boundary, distSqr, s);
        }
    }
};
/**
 * Checks if the boundary data is still valid.
 * @param boundary The local boundary to check
 * @param navMesh Navigation mesh
 * @param filter Query filter
 * @returns True if valid
 */
const isLocalBoundaryValid = (boundary, navMesh, filter) => {
    if (boundary.polys.length === 0) {
        return false;
    }
    // check that all polygons still pass query filter
    for (const polyRef of boundary.polys) {
        if (!isValidNodeRef(navMesh, polyRef)) {
            return false;
        }
        // check filter if available
        if (!filter.passFilter(polyRef, navMesh)) {
            return false;
        }
    }
    return true;
};

var localBoundary = /*#__PURE__*/Object.freeze({
    __proto__: null,
    create: create$2,
    isLocalBoundaryValid: isLocalBoundaryValid,
    resetLocalBoundary: resetLocalBoundary,
    updateLocalBoundary: updateLocalBoundary
});

const DT_PI = Math.PI;
const DT_MAX_PATTERN_DIVS = 32; // Max number of adaptive divs
const DT_MAX_PATTERN_RINGS = 4; // Max number of adaptive rings
/**
 * Creates a new obstacle avoidance query.
 */
const createObstacleAvoidanceQuery = (maxCircles, maxSegments) => {
    // pre-allocate obstacle objects
    const circles = [];
    const segments = [];
    for (let i = 0; i < maxCircles; i++) {
        circles.push({
            p: [0, 0, 0],
            vel: [0, 0, 0],
            dvel: [0, 0, 0],
            rad: 0,
            dp: [0, 0, 0],
            np: [0, 0, 0],
        });
    }
    for (let i = 0; i < maxSegments; i++) {
        segments.push({
            p: [0, 0, 0],
            q: [0, 0, 0],
            touch: false,
        });
    }
    return {
        circles,
        segments,
        maxCircles,
        maxSegments,
        circleCount: 0,
        segmentCount: 0,
        params: {
            velBias: 0.4,
            weightDesVel: 2.0,
            weightCurVel: 0.75,
            weightSide: 0.75,
            weightToi: 2.5,
            horizTime: 2.5,
            gridSize: 33,
            adaptiveDivs: 7,
            adaptiveRings: 2,
            adaptiveDepth: 5,
        },
        invHorizTime: 0,
        vmax: 0,
        invVmax: 0,
        pattern: new Float32Array((DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2),
    };
};
/**
 * Creates debug data for obstacle avoidance.
 */
const createObstacleAvoidanceDebugData = () => ({
    samples: [],
});
/**
 * Resets the obstacle avoidance query.
 */
const resetObstacleAvoidanceQuery = (query) => {
    query.circleCount = 0;
    query.segmentCount = 0;
};
/**
 * Resets debug data.
 */
const resetObstacleAvoidanceDebugData = (debug) => {
    debug.samples.length = 0;
};
/**
 * Adds a circular obstacle to the query.
 */
const addCircleObstacle = (query, pos, rad, vel, dvel) => {
    if (query.circleCount >= query.maxCircles)
        return;
    const circle = query.circles[query.circleCount];
    // Copy data to pre-allocated object
    vec3.copy(circle.p, pos);
    vec3.copy(circle.vel, vel);
    vec3.copy(circle.dvel, dvel);
    circle.rad = rad;
    // Reset computed values
    vec3.set(circle.dp, 0, 0, 0);
    vec3.set(circle.np, 0, 0, 0);
    query.circleCount++;
};
/**
 * Adds a segment obstacle to the query.
 */
const addSegmentObstacle = (query, p, q) => {
    if (query.segmentCount >= query.maxSegments)
        return;
    const segment = query.segments[query.segmentCount];
    // Copy data to pre-allocated object
    vec3.copy(segment.p, p);
    vec3.copy(segment.q, q);
    segment.touch = false;
    query.segmentCount++;
};
/**
 * Helper function to calculate 2D triangle area.
 */
const triArea2D = (a, b, c) => {
    const abx = b[0] - a[0];
    const abz = b[2] - a[2];
    const acx = c[0] - a[0];
    const acz = c[2] - a[2];
    return acx * abz - abx * acz;
};
/**
 * Helper function to calculate 2D dot product.
 */
const vdot2D = (a, b) => a[0] * b[0] + a[2] * b[2];
/**
 * Helper function to calculate 2D perpendicular dot product.
 */
const vperp2D = (a, b) => a[0] * b[2] - a[2] * b[0];
/**
 * Helper function to calculate 2D distance.
 */
const vdist2D = (a, b) => {
    const dx = b[0] - a[0];
    const dz = b[2] - a[2];
    return Math.sqrt(dx * dx + dz * dz);
};
/**
 * Helper function to calculate squared distance from point to segment in 2D.
 */
const distancePtSegSqr2D = (pt, p, q) => {
    const pqx = q[0] - p[0];
    const pqz = q[2] - p[2];
    const dx = pt[0] - p[0];
    const dz = pt[2] - p[2];
    const d = pqx * pqx + pqz * pqz;
    let t = pqx * dx + pqz * dz;
    if (d > 0)
        t /= d;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;
    const nearestX = p[0] + t * pqx;
    const nearestZ = p[2] + t * pqz;
    const distX = pt[0] - nearestX;
    const distZ = pt[2] - nearestZ;
    return distX * distX + distZ * distZ;
};
const _sweepCircleCircle_s = vec3.create();
/**
 * Sweep test between two circles.
 */
const sweepCircleCircle = (c0, r0, v, c1, r1, out) => {
    const EPS = 0.0001;
    const s = _sweepCircleCircle_s;
    const sx = c1[0] - c0[0];
    const sz = c1[2] - c0[2];
    s[0] = sx;
    s[1] = 0; // Not used, but keep vector valid
    s[2] = sz;
    const r = r0 + r1;
    // vdot2D(s, s)
    const sSqr = sx * sx + sz * sz;
    const c = sSqr - r * r;
    // vdot2D(v, v)
    const a = v[0] * v[0] + v[2] * v[2];
    if (a < EPS) {
        out.hit = false;
        out.tmin = 0;
        out.tmax = 0;
        return;
    }
    // vdot2D(v, s)
    const b = v[0] * sx + v[2] * sz;
    const d = b * b - a * c;
    if (d < 0.0) {
        out.hit = false;
        out.tmin = 0;
        out.tmax = 0;
        return;
    }
    const invA = 1.0 / a;
    const rd = Math.sqrt(d);
    out.hit = true;
    out.tmin = (b - rd) * invA;
    out.tmax = (b + rd) * invA;
};
const _intersectRaySegment_v = vec3.create();
const _intersectRaySegment_w = vec3.create();
/**
 * Ray-segment intersection test.
 */
const intersectRaySegment = (ap, u, bp, bq, out) => {
    const v = _intersectRaySegment_v;
    v[0] = bq[0] - bp[0];
    v[1] = bq[1] - bp[1];
    v[2] = bq[2] - bp[2];
    const w = _intersectRaySegment_w;
    w[0] = ap[0] - bp[0];
    w[1] = ap[1] - bp[1];
    w[2] = ap[2] - bp[2];
    const d = vperp2D(u, v);
    if (Math.abs(d) < 1e-6) {
        out.hit = false;
        out.t = 0;
        return;
    }
    const invD = 1.0 / d;
    const t = vperp2D(v, w) * invD;
    if (t < 0 || t > 1) {
        out.hit = false;
        out.t = 0;
        return;
    }
    const s = vperp2D(u, w) * invD;
    if (s < 0 || s > 1) {
        out.hit = false;
        out.t = 0;
        return;
    }
    out.hit = true;
    out.t = t;
};
const _prepareObstacles_orig = vec3.create();
const _prepareObstacles_dv = vec3.create();
/**
 * Prepares obstacles for sampling by calculating side information.
 */
const prepareObstacles = (query, pos, dvel) => {
    // prepare circular obstacles
    for (let i = 0; i < query.circleCount; i++) {
        const cir = query.circles[i];
        // side calculation
        const pa = pos;
        const pb = cir.p;
        const orig = vec3.set(_prepareObstacles_orig, 0, 0, 0);
        vec3.sub(cir.dp, pb, pa);
        vec3.normalize(cir.dp, cir.dp);
        const dv = vec3.sub(_prepareObstacles_dv, cir.dvel, dvel);
        const a = triArea2D(orig, cir.dp, dv);
        if (a < 0.01) {
            cir.np[0] = -cir.dp[2];
            cir.np[1] = 0;
            cir.np[2] = cir.dp[0];
        }
        else {
            cir.np[0] = cir.dp[2];
            cir.np[1] = 0;
            cir.np[2] = -cir.dp[0];
        }
    }
    // Prepare segment obstacles
    for (let i = 0; i < query.segmentCount; i++) {
        const seg = query.segments[i];
        // Precalc if the agent is really close to the segment
        const r = 0.01;
        const distSqr = distancePtSegSqr2D(pos, seg.p, seg.q);
        seg.touch = distSqr < r * r;
    }
};
/**
 * Copies parameters to avoid object allocation.
 */
const copyParams = (dest, src) => {
    dest.velBias = src.velBias;
    dest.weightDesVel = src.weightDesVel;
    dest.weightCurVel = src.weightCurVel;
    dest.weightSide = src.weightSide;
    dest.weightToi = src.weightToi;
    dest.horizTime = src.horizTime;
    dest.gridSize = src.gridSize;
    dest.adaptiveDivs = src.adaptiveDivs;
    dest.adaptiveRings = src.adaptiveRings;
    dest.adaptiveDepth = src.adaptiveDepth;
};
const _vab = vec3.create();
const _sdir = vec3.create();
const _snorm = vec3.create();
const _sweepResult = { hit: false, tmin: 0, tmax: 0 };
const _intersectionResult = { hit: false, t: 0 };
/**
 * Process a velocity sample and calculate its penalty.
 */
const processSample = (query, vcand, cs, pos, rad, vel, dvel, minPenalty, debug) => {
    const params = query.params;
    // penalty for straying away from desired and current velocities
    const vpen = params.weightDesVel * (vdist2D(vcand, dvel) * query.invVmax);
    const vcpen = params.weightCurVel * (vdist2D(vcand, vel) * query.invVmax);
    // find threshold hit time to bail out based on early out penalty
    const minPen = minPenalty - vpen - vcpen;
    const tThreshold = (params.weightToi / minPen - 0.1) * params.horizTime;
    if (tThreshold - params.horizTime > -Number.EPSILON) {
        return minPenalty; // already too much
    }
    // find min time of impact and exit amongst all obstacles
    let tmin = params.horizTime;
    let side = 0;
    let nside = 0;
    // check circular obstacles
    for (let i = 0; i < query.circleCount; i++) {
        const cir = query.circles[i];
        // RVO (Reciprocal Velocity Obstacles)
        // vec3.scale(vab, vcand, 2);
        // vec3.sub(vab, vab, vel);
        // vec3.sub(vab, vab, cir.vel);
        const vab = _vab;
        vab[0] = vcand[0] * 2 - vel[0] - cir.vel[0];
        vab[1] = vcand[1] * 2 - vel[1] - cir.vel[1];
        vab[2] = vcand[2] * 2 - vel[2] - cir.vel[2];
        // side bias
        side += Math.max(0, Math.min(1, Math.min(vdot2D(cir.dp, vab) * 0.5 + 0.5, vdot2D(cir.np, vab) * 2)));
        nside++;
        sweepCircleCircle(pos, rad, vab, cir.p, cir.rad, _sweepResult);
        if (!_sweepResult.hit)
            continue;
        let htmin = _sweepResult.tmin;
        const htmax = _sweepResult.tmax;
        // handle overlapping obstacles
        if (htmin < 0.0 && htmax > 0.0) {
            // avoid more when overlapped
            htmin = -htmin * 0.5;
        }
        if (htmin >= 0.0) {
            // the closest obstacle is somewhere ahead of us
            if (htmin < tmin) {
                tmin = htmin;
                if (tmin < tThreshold) {
                    return minPenalty;
                }
            }
        }
    }
    // check segment obstacles
    for (let i = 0; i < query.segmentCount; i++) {
        const seg = query.segments[i];
        let htmin = 0;
        if (seg.touch) {
            // special case when agent is very close to segment
            const sdir = vec3.set(_sdir, seg.q[0] - seg.p[0], seg.q[1] - seg.p[1], seg.q[2] - seg.p[2]);
            const snorm = vec3.set(_snorm, -sdir[2], sdir[1], sdir[0]);
            // if the velocity is pointing towards the segment, no collision.
            if (vdot2D(snorm, vcand) < 0.0)
                continue;
            // else immediate collision.
            htmin = 0.0;
        }
        else {
            intersectRaySegment(pos, vcand, seg.p, seg.q, _intersectionResult);
            if (!_intersectionResult.hit)
                continue;
            htmin = _intersectionResult.t;
        }
        // avoid less when facing walls
        htmin *= 2.0;
        // track nearest obstacle
        if (htmin < tmin) {
            tmin = htmin;
            if (tmin < tThreshold) {
                return minPenalty;
            }
        }
    }
    // normalize side bias
    if (nside > 0) {
        side /= nside;
    }
    const spen = params.weightSide * side;
    const tpen = params.weightToi * (1.0 / (0.1 + tmin * query.invHorizTime));
    const penalty = vpen + vcpen + spen + tpen;
    // store debug info
    if (debug) {
        debug.samples.push({
            vel: vec3.clone(vcand),
            ssize: cs,
            pen: penalty,
            vpen,
            vcpen,
            spen,
            tpen,
        });
    }
    return penalty;
};
const _sampleVelocityGrid_vcand = vec3.create();
/**
 * Sample velocity using grid-based approach.
 */
const sampleVelocityGrid = (query, pos, rad, vmax, vel, dvel, params, debug) => {
    prepareObstacles(query, pos, dvel);
    copyParams(query.params, params);
    query.invHorizTime = 1.0 / query.params.horizTime;
    query.vmax = vmax;
    query.invVmax = vmax > 0 ? 1.0 / vmax : Number.MAX_VALUE;
    const nvel = [0, 0, 0];
    if (debug) {
        resetObstacleAvoidanceDebugData(debug);
    }
    const cvx = dvel[0] * params.velBias;
    const cvz = dvel[2] * params.velBias;
    const cs = (vmax * 2 * (1 - params.velBias)) / (params.gridSize - 1);
    const half = ((params.gridSize - 1) * cs) * 0.5;
    let minPenalty = Number.MAX_VALUE;
    let ns = 0;
    // pre-compute vmax squared for bounds checking
    const vmaxPlusHalfCs = vmax + cs / 2;
    const vmaxSqr = vmaxPlusHalfCs * vmaxPlusHalfCs;
    for (let y = 0; y < params.gridSize; ++y) {
        for (let x = 0; x < params.gridSize; ++x) {
            const vcand = _sampleVelocityGrid_vcand;
            vcand[0] = cvx + x * cs - half;
            vcand[1] = 0;
            vcand[2] = cvz + y * cs - half;
            if (vcand[0] * vcand[0] + vcand[2] * vcand[2] > vmaxSqr) {
                continue;
            }
            const penalty = processSample(query, vcand, cs, pos, rad, vel, dvel, minPenalty, debug);
            ns++;
            if (penalty < minPenalty) {
                minPenalty = penalty;
                vec3.copy(nvel, vcand);
            }
        }
    }
    return { nvel, samples: ns };
};
/**
 * Normalize a 2D vector (ignoring Y component).
 */
const normalize2D = (v) => {
    const d = Math.sqrt(v[0] * v[0] + v[2] * v[2]);
    if (d === 0)
        return;
    const invD = 1.0 / d;
    v[0] *= invD;
    v[2] *= invD;
};
/**
 * Rotate a 2D vector (ignoring Y component).
 */
const rotate2D = (dest, v, ang) => {
    const c = Math.cos(ang);
    const s = Math.sin(ang);
    dest[0] = v[0] * c - v[2] * s;
    dest[2] = v[0] * s + v[2] * c;
    dest[1] = v[1];
};
const _sampleVelocityAdaptive_ddir = vec3.create();
const _sampleVelocityAdaptive_ddir2 = vec3.create();
const _sampleVelocityAdaptive_res = vec3.create();
const _sampleVelocityAdaptive_bvel = vec3.create();
const _sampleVelocityAdaptive_vcand = vec3.create();
/**
 * Sample velocity using adaptive approach.
 */
const sampleVelocityAdaptive = (query, pos, rad, vmax, vel, dvel, params, outVelocity, debug) => {
    prepareObstacles(query, pos, dvel);
    copyParams(query.params, params);
    query.invHorizTime = 1.0 / query.params.horizTime;
    query.vmax = vmax;
    query.invVmax = vmax > 0 ? 1.0 / vmax : Number.MAX_VALUE;
    if (debug) {
        resetObstacleAvoidanceDebugData(debug);
    }
    // build sampling pattern aligned to desired velocity
    const pat = query.pattern;
    let npat = 0;
    const ndivs = Math.max(1, Math.min(query.params.adaptiveDivs, DT_MAX_PATTERN_DIVS));
    const nrings = Math.max(1, Math.min(query.params.adaptiveRings, DT_MAX_PATTERN_RINGS));
    const depth = query.params.adaptiveDepth;
    const da = (1.0 / ndivs) * DT_PI * 2;
    const ca = Math.cos(da);
    const sa = Math.sin(da);
    // desired direction - use pre-allocated vectors to avoid cloning
    const ddir = _sampleVelocityAdaptive_ddir;
    vec3.copy(ddir, dvel);
    normalize2D(ddir);
    const ddir2 = _sampleVelocityAdaptive_ddir2;
    rotate2D(ddir2, ddir, da * 0.5); // rotated by da/2
    // always add sample at zero
    pat[npat * 2] = 0;
    pat[npat * 2 + 1] = 0;
    npat++;
    for (let j = 0; j < nrings; ++j) {
        const r = (nrings - j) / nrings;
        // use pattern similar to C++: ddir[(j%2)*3] selects between ddir and ddir2
        const baseDir = j % 2 === 0 ? ddir : ddir2;
        pat[npat * 2] = baseDir[0] * r;
        pat[npat * 2 + 1] = baseDir[2] * r;
        let last1 = npat * 2; // Points to current element
        let last2 = last1; // Both point to same location initially
        npat++;
        for (let i = 1; i < ndivs - 1; i += 2) {
            // Get next point on the "right" (rotate CW)
            pat[npat * 2] = pat[last1] * ca + pat[last1 + 1] * sa;
            pat[npat * 2 + 1] = -pat[last1] * sa + pat[last1 + 1] * ca;
            // Get next point on the "left" (rotate CCW)  
            pat[npat * 2 + 2] = pat[last2] * ca - pat[last2 + 1] * sa;
            pat[npat * 2 + 3] = pat[last2] * sa + pat[last2 + 1] * ca;
            last1 = npat * 2; // Point to current "right" element
            last2 = last1 + 2; // Point to current "left" element
            npat += 2;
        }
        if ((ndivs & 1) === 0) {
            pat[npat * 2] = pat[last2] * ca - pat[last2 + 1] * sa;
            pat[npat * 2 + 1] = pat[last2] * sa + pat[last2 + 1] * ca;
            npat++;
        }
    }
    // start sampling
    let cr = vmax * (1.0 - query.params.velBias);
    const res = _sampleVelocityAdaptive_res;
    res[0] = dvel[0] * query.params.velBias;
    res[1] = 0;
    res[2] = dvel[2] * query.params.velBias;
    let ns = 0;
    // pre-compute vmax squared for bounds checking
    const vmaxPlusEpsilon = vmax + 0.001;
    const vmaxSqr = vmaxPlusEpsilon * vmaxPlusEpsilon;
    for (let k = 0; k < depth; ++k) {
        let minPenalty = Number.MAX_VALUE;
        const bvel = _sampleVelocityAdaptive_bvel;
        vec3.set(bvel, 0, 0, 0);
        // Cache cr / 10 for this depth iteration
        const crOverTen = cr * 0.1;
        for (let i = 0; i < npat; ++i) {
            const vcand = _sampleVelocityAdaptive_vcand;
            vcand[0] = res[0] + pat[i * 2] * cr;
            vcand[1] = 0;
            vcand[2] = res[2] + pat[i * 2 + 1] * cr;
            if (vcand[0] * vcand[0] + vcand[2] * vcand[2] > vmaxSqr) {
                continue;
            }
            const penalty = processSample(query, vcand, crOverTen, pos, rad, vel, dvel, minPenalty, debug);
            ns++;
            if (penalty < minPenalty) {
                minPenalty = penalty;
                vec3.copy(bvel, vcand);
            }
        }
        vec3.copy(res, bvel);
        cr *= 0.5;
    }
    vec3.copy(outVelocity, res);
    return ns;
};

var obstacleAvoidance = /*#__PURE__*/Object.freeze({
    __proto__: null,
    addCircleObstacle: addCircleObstacle,
    addSegmentObstacle: addSegmentObstacle,
    createObstacleAvoidanceDebugData: createObstacleAvoidanceDebugData,
    createObstacleAvoidanceQuery: createObstacleAvoidanceQuery,
    resetObstacleAvoidanceDebugData: resetObstacleAvoidanceDebugData,
    resetObstacleAvoidanceQuery: resetObstacleAvoidanceQuery,
    sampleVelocityAdaptive: sampleVelocityAdaptive,
    sampleVelocityGrid: sampleVelocityGrid
});

const create$1 = () => ({
    position: [0, 0, 0],
    target: [0, 0, 0],
    path: [],
});
const reset = (corridor, ref, position) => {
    vec3.copy(corridor.position, position);
    vec3.copy(corridor.target, position);
    corridor.path = [ref];
};
const setPath = (corridor, target, path) => {
    vec3.copy(corridor.target, target);
    corridor.path = path;
};
const mergeStartMoved = (currentPath, visited) => {
    if (visited.length === 0)
        return currentPath;
    let furthestPath = -1;
    let furthestVisited = -1;
    // find furthest common polygon
    for (let i = currentPath.length - 1; i >= 0; i--) {
        for (let j = visited.length - 1; j >= 0; j--) {
            if (currentPath[i] === visited[j]) {
                furthestPath = i;
                furthestVisited = j;
                break;
            }
        }
        if (furthestPath !== -1)
            break;
    }
    // if no intersection found, just return current path
    if (furthestPath === -1 || furthestVisited === -1) {
        return currentPath;
    }
    // concatenate paths
    const req = visited.length - furthestVisited;
    const orig = Math.min(furthestPath + 1, currentPath.length);
    const size = Math.max(0, currentPath.length - orig);
    const newPath = [];
    // store visited polygons (in reverse order)
    for (let i = 0; i < req; i++) {
        newPath[i] = visited[visited.length - 1 - i];
    }
    // add remaining current path
    if (size > 0) {
        for (let i = 0; i < size; i++) {
            newPath[req + i] = currentPath[orig + i];
        }
    }
    return newPath;
};
const mergeStartShortcut = (currentPath, visited) => {
    if (visited.length === 0)
        return currentPath;
    let furthestPath = -1;
    let furthestVisited = -1;
    // find furthest common polygon
    for (let i = currentPath.length - 1; i >= 0; i--) {
        for (let j = visited.length - 1; j >= 0; j--) {
            if (currentPath[i] === visited[j]) {
                furthestPath = i;
                furthestVisited = j;
                break;
            }
        }
        if (furthestPath !== -1)
            break;
    }
    // if no intersection found, just return current path
    if (furthestPath === -1 || furthestVisited === -1) {
        return currentPath;
    }
    // concatenate paths
    const req = furthestVisited;
    if (req <= 0) {
        return currentPath;
    }
    const orig = furthestPath;
    const size = Math.max(0, currentPath.length - orig);
    const newPath = [];
    // store visited polygons (not reversed like mergeStartMoved)
    for (let i = 0; i < req; i++) {
        newPath[i] = visited[i];
    }
    // add remaining current path
    if (size > 0) {
        for (let i = 0; i < size; i++) {
            newPath[req + i] = currentPath[orig + i];
        }
    }
    return newPath;
};
const movePosition = (corridor, newPos, navMesh, filter) => {
    if (corridor.path.length === 0)
        return false;
    const result = moveAlongSurface(navMesh, corridor.path[0], corridor.position, newPos, filter);
    if (result.success) {
        corridor.path = mergeStartMoved(corridor.path, result.visited);
        vec3.copy(corridor.position, result.position);
        return true;
    }
    return false;
};
const MIN_TARGET_DIST = 0.01;
const findCorners = (corridor, navMesh, maxCorners) => {
    if (corridor.path.length === 0)
        return false;
    const straightPathResult = findStraightPath(navMesh, corridor.position, corridor.target, corridor.path, maxCorners);
    if (!straightPathResult.success || straightPathResult.path.length === 0) {
        return false;
    }
    let corners = straightPathResult.path;
    // prune points in the beginning of the path which are too close
    while (corners.length > 0) {
        const firstCorner = corners[0];
        const distance = vec3.distance(corridor.position, firstCorner.position);
        // if the first corner is far enough, we're done pruning
        if (firstCorner.type === NodeType.OFFMESH || distance > MIN_TARGET_DIST) {
            break;
        }
        // remove the first corner as it's too close
        corners = corners.slice(1);
    }
    // prune points after an offmesh connection
    let firstOffMeshConnectionIndex = -1;
    for (let i = 0; i < corners.length; i++) {
        if (corners[i].type === NodeType.OFFMESH) {
            firstOffMeshConnectionIndex = i;
            break;
        }
    }
    if (firstOffMeshConnectionIndex !== -1) {
        corners = corners.slice(0, firstOffMeshConnectionIndex + 1);
    }
    return corners;
};
const corridorIsValid = (corridor, maxLookAhead, navMesh, filter) => {
    const n = Math.min(corridor.path.length, maxLookAhead);
    // check nodes are still valid and pass query filter
    for (let i = 0; i < n; i++) {
        const nodeRef = corridor.path[i];
        if (!isValidNodeRef(navMesh, nodeRef) || !filter.passFilter(nodeRef, navMesh)) {
            return false;
        }
    }
    return true;
};
const fixPathStart = (corridor, safeRef, safePos) => {
    vec3.copy(corridor.position, safePos);
    if (corridor.path.length < 3 && corridor.path.length > 0) {
        const lastPoly = corridor.path[corridor.path.length - 1];
        corridor.path[2] = lastPoly;
        corridor.path[0] = safeRef;
        corridor.path[1] = INVALID_NODE_REF;
        corridor.path.length = 3;
    }
    else {
        corridor.path[0] = safeRef;
        corridor.path[1] = INVALID_NODE_REF;
    }
    return true;
};
const moveOverOffMeshConnection = (corridor, offMeshNodeRef, navMesh) => {
    if (corridor.path.length === 0)
        return false;
    // advance the path up to and over the off-mesh connection.
    let prevNodeRef = null;
    let nodeRef = corridor.path[0];
    let i = 0;
    while (i < corridor.path.length && nodeRef !== offMeshNodeRef) {
        prevNodeRef = nodeRef;
        i++;
        if (i < corridor.path.length) {
            nodeRef = corridor.path[i];
        }
    }
    if (i === corridor.path.length) {
        // could not find the off mesh connection node
        return false;
    }
    // prune path - remove the elements from 0 up to and including the off-mesh connection
    corridor.path = corridor.path.slice(i + 1);
    if (!prevNodeRef) {
        return false;
    }
    // get the off-mesh connection
    const { offMeshConnectionId } = getNodeByRef(navMesh, offMeshNodeRef);
    const offMeshConnection = navMesh.offMeshConnections[offMeshConnectionId];
    const offMeshConnectionAttachment = navMesh.offMeshConnectionAttachments[offMeshConnectionId];
    if (!offMeshConnection || !offMeshConnectionAttachment)
        return false;
    // determine which end we're moving to
    const onStart = offMeshConnectionAttachment.startPolyNode === prevNodeRef;
    const endPosition = onStart ? offMeshConnection.end : offMeshConnection.start;
    const endNodeRef = onStart ? offMeshConnectionAttachment.endPolyNode : offMeshConnectionAttachment.startPolyNode;
    vec3.copy(corridor.position, endPosition);
    return {
        startPosition: onStart ? offMeshConnection.start : offMeshConnection.end,
        endPosition,
        endNodeRef,
        prevNodeRef,
        offMeshNodeRef,
    };
};
/**
 * Attempts to optimize the path using a local area search (partial replanning).
 *
 * Inaccurate locomotion or dynamic obstacle avoidance can force the agent position significantly
 * outside the original corridor. Over time this can result in the formation of a non-optimal corridor.
 * This function will use a local area path search to try to re-optimize the corridor.
 *
 * The more inaccurate the agent movement, the more beneficial this function becomes.
 * Simply adjust the frequency of the call to match the needs of the agent.
 *
 * @param corridor the path corridor
 * @param navMesh the navigation mesh
 * @param filter the query filter
 * @returns true if the path was optimized, false otherwise
 */
const optimizePathTopology = (corridor, navMesh, filter) => {
    if (corridor.path.length < 3) {
        return false;
    }
    const MAX_ITER = 32;
    const query = createSlicedNodePathQuery();
    // do a local area search from start to end
    initSlicedFindNodePath(navMesh, query, corridor.path[0], corridor.path[corridor.path.length - 1], corridor.position, corridor.target, filter);
    updateSlicedFindNodePath(navMesh, query, MAX_ITER);
    const result = finalizeSlicedFindNodePathPartial(navMesh, query, corridor.path);
    if ((query.status & SlicedFindNodePathStatusFlags.SUCCESS) !== 0 && result.path.length > 0) {
        // merge the optimized path with the corridor using shortcut merge
        corridor.path = mergeStartShortcut(corridor.path, result.path);
        return true;
    }
    return false;
};
const _optimizePathVisibility_goal = vec3.create();
const _optimizePathVisibility_delta = vec3.create();
/**
 * Attempts to optimize the path if the specified point is visible from the current position.
 *
 * Inaccurate locomotion or dynamic obstacle avoidance can force the agent position significantly
 * outside the original corridor. Over time this can result in the formation of a non-optimal corridor.
 * Non-optimal paths can also form near the corners of tiles.
 *
 * This function uses an efficient local visibility search to try to optimize the corridor
 * between the current position and the target.
 *
 * The corridor will change only if the target is visible from the current position and moving
 * directly toward the point is better than following the existing path.
 *
 * The more inaccurate the agent movement, the more beneficial this function becomes.
 * Simply adjust the frequency of the call to match the needs of the agent.
 *
 * This function is not suitable for long distance searches.
 *
 * @param corridor the path corridor
 * @param next the point to search toward
 * @param pathOptimizationRange the maximum range to search
 * @param navMesh the navigation mesh
 * @param filter the query filter
 */
const optimizePathVisibility = (corridor, next, pathOptimizationRange, navMesh, filter) => {
    if (corridor.path.length === 0) {
        return;
    }
    // Clamp the ray to max distance.
    const goal = vec3.copy(_optimizePathVisibility_goal, next);
    const dx = goal[0] - corridor.position[0];
    const dz = goal[2] - corridor.position[2];
    let dist = Math.sqrt(dx * dx + dz * dz);
    // If too close to the goal, do not try to optimize.
    if (dist < 0.01) {
        return;
    }
    // Overshoot a little. This helps to optimize open fields in tiled meshes.
    dist = Math.min(dist + 0.01, pathOptimizationRange);
    // Adjust ray length.
    const delta = vec3.subtract(_optimizePathVisibility_delta, goal, corridor.position);
    vec3.scaleAndAdd(goal, corridor.position, delta, pathOptimizationRange / dist);
    const result = raycast(navMesh, corridor.path[0], corridor.position, goal, filter);
    if (result.path.length > 1 && result.t > 0.99) {
        corridor.path = mergeStartShortcut(corridor.path, result.path);
    }
};

var pathCorridor = /*#__PURE__*/Object.freeze({
    __proto__: null,
    corridorIsValid: corridorIsValid,
    create: create$1,
    findCorners: findCorners,
    fixPathStart: fixPathStart,
    mergeStartMoved: mergeStartMoved,
    mergeStartShortcut: mergeStartShortcut,
    moveOverOffMeshConnection: moveOverOffMeshConnection,
    movePosition: movePosition,
    optimizePathTopology: optimizePathTopology,
    optimizePathVisibility: optimizePathVisibility,
    reset: reset,
    setPath: setPath
});

var AgentState;
(function (AgentState) {
    AgentState[AgentState["INVALID"] = 0] = "INVALID";
    AgentState[AgentState["WALKING"] = 1] = "WALKING";
    AgentState[AgentState["OFFMESH"] = 2] = "OFFMESH";
})(AgentState || (AgentState = {}));
var AgentTargetState;
(function (AgentTargetState) {
    AgentTargetState[AgentTargetState["NONE"] = 0] = "NONE";
    AgentTargetState[AgentTargetState["FAILED"] = 1] = "FAILED";
    AgentTargetState[AgentTargetState["VALID"] = 2] = "VALID";
    AgentTargetState[AgentTargetState["REQUESTING"] = 3] = "REQUESTING";
    AgentTargetState[AgentTargetState["WAITING_FOR_QUEUE"] = 4] = "WAITING_FOR_QUEUE";
    AgentTargetState[AgentTargetState["WAITING_FOR_PATH"] = 5] = "WAITING_FOR_PATH";
    AgentTargetState[AgentTargetState["VELOCITY"] = 6] = "VELOCITY";
})(AgentTargetState || (AgentTargetState = {}));
var CrowdUpdateFlags;
(function (CrowdUpdateFlags) {
    CrowdUpdateFlags[CrowdUpdateFlags["ANTICIPATE_TURNS"] = 1] = "ANTICIPATE_TURNS";
    CrowdUpdateFlags[CrowdUpdateFlags["OBSTACLE_AVOIDANCE"] = 2] = "OBSTACLE_AVOIDANCE";
    CrowdUpdateFlags[CrowdUpdateFlags["SEPARATION"] = 4] = "SEPARATION";
    CrowdUpdateFlags[CrowdUpdateFlags["OPTIMIZE_VIS"] = 8] = "OPTIMIZE_VIS";
    CrowdUpdateFlags[CrowdUpdateFlags["OPTIMIZE_TOPO"] = 16] = "OPTIMIZE_TOPO";
})(CrowdUpdateFlags || (CrowdUpdateFlags = {}));
/** Sensible default obstacle avoidance parameters */
const DEFAULT_OBSTACLE_AVOIDANCE_PARAMS = {
    velBias: 0.4,
    weightDesVel: 2.0,
    weightCurVel: 0.75,
    weightSide: 0.75,
    weightToi: 2.5,
    horizTime: 2.5,
    gridSize: 33,
    adaptiveDivs: 7,
    adaptiveRings: 2,
    adaptiveDepth: 5,
};
/**
 * Creates a new crowd
 * @param maxAgentRadius the maximum agent radius in the crowd
 * @returns the created crowd
 */
const create = (maxAgentRadius) => {
    return {
        agents: {},
        agentIdCounter: 0,
        maxAgentRadius,
        agentPlacementHalfExtents: [maxAgentRadius, maxAgentRadius, maxAgentRadius],
        maxIterationsPerUpdate: 600,
        maxIterationsPerAgent: 200,
        quickSearchIterations: 20,
    };
};
/**
 * Adds an agent to the crowd.
 * @param crowd the crowd
 * @param position the initial position of the agent
 * @param agentParams the parameters for the agent
 * @returns the ID of the added agent
 */
const addAgent = (crowd, navMesh, position, agentParams) => {
    const agentId = String(crowd.agentIdCounter++);
    const agent = {
        radius: agentParams.radius,
        height: agentParams.height,
        maxAcceleration: agentParams.maxAcceleration,
        maxSpeed: agentParams.maxSpeed,
        collisionQueryRange: agentParams.collisionQueryRange,
        pathOptimizationRange: agentParams.pathOptimizationRange ?? agentParams.radius * 30.0,
        separationWeight: agentParams.separationWeight,
        updateFlags: agentParams.updateFlags,
        queryFilter: agentParams.queryFilter,
        obstacleAvoidance: agentParams.obstacleAvoidance ?? DEFAULT_OBSTACLE_AVOIDANCE_PARAMS,
        autoTraverseOffMeshConnections: agentParams.autoTraverseOffMeshConnections ?? true,
        state: AgentState.WALKING,
        corridor: create$1(),
        slicedQuery: createSlicedNodePathQuery(),
        boundary: create$2(),
        obstacleAvoidanceQuery: createObstacleAvoidanceQuery(32, 32),
        obstacleAvoidanceDebugData: agentParams.debugObstacleAvoidance ? createObstacleAvoidanceDebugData() : undefined,
        neis: [],
        corners: [],
        position,
        desiredSpeed: 0,
        desiredVelocity: [0, 0, 0],
        newVelocity: [0, 0, 0],
        velocity: [0, 0, 0],
        displacement: [0, 0, 0],
        targetState: AgentTargetState.NONE,
        targetRef: null,
        targetPosition: [0, 0, 0],
        targetReplan: false,
        targetPathfindingTime: 0,
        targetPathIsPartial: false,
        topologyOptTime: 0,
        offMeshAnimation: null,
    };
    crowd.agents[agentId] = agent;
    // find nearest position on navmesh and place the agent there
    const nearestPolyResult = createFindNearestPolyResult();
    findNearestPoly(nearestPolyResult, navMesh, position, crowd.agentPlacementHalfExtents, agent.queryFilter);
    let nearestPos = position;
    let nearestRef = INVALID_NODE_REF;
    if (nearestPolyResult.success) {
        nearestPos = nearestPolyResult.position;
        nearestRef = nearestPolyResult.nodeRef;
    }
    // reset corridor with the found (or invalid) reference
    vec3.copy(agent.position, nearestPos);
    reset(agent.corridor, nearestRef, nearestPos);
    // set agent state based on whether we found a valid polygon
    if (nearestRef !== INVALID_NODE_REF) {
        agent.state = AgentState.WALKING;
    }
    else {
        agent.state = AgentState.INVALID;
    }
    return agentId;
};
/**
 * Removes an agent from the crowd.
 * @param crowd the crowd
 * @param agentId the ID of the agent
 * @returns true if the agent was removed, false otherwise
 */
const removeAgent = (crowd, agentId) => {
    if (crowd.agents[agentId]) {
        delete crowd.agents[agentId];
        return true;
    }
    return false;
};
/**
 * Requests a move target for an agent.
 * @param crowd the crowd
 * @param agentId the ID of the agent
 * @param targetRef the target reference
 * @param targetPos the target position
 * @returns true if the move target was set, false otherwise
 */
const requestMoveTarget = (crowd, agentId, targetRef, targetPos) => {
    const agent = crowd.agents[agentId];
    if (!agent)
        return false;
    agent.targetRef = targetRef;
    vec3.copy(agent.targetPosition, targetPos);
    // if the agent already has a corridor path, this is a replan
    agent.targetReplan = agent.corridor.path.length > 0;
    agent.targetState = AgentTargetState.REQUESTING;
    agent.targetPathIsPartial = false;
    agent.targetPathfindingTime = 0; // Reset timer for new request
    return true;
};
const requestMoveTargetReplan = (crowd, agentId, targetRef, targetPos) => {
    const agent = crowd.agents[agentId];
    if (!agent)
        return false;
    agent.targetRef = targetRef;
    vec3.copy(agent.targetPosition, targetPos);
    agent.targetReplan = true;
    agent.targetState = AgentTargetState.REQUESTING;
    return true;
};
/**
 * Request a move velocity for an agent.
 * @param crowd the crowd
 * @param agentId the ID of the agent
 * @param velocity the desired velocity
 * @returns true if the move velocity was set, false otherwise
 */
const requestMoveVelocity = (crowd, agentId, velocity) => {
    const agent = crowd.agents[agentId];
    if (!agent)
        return false;
    vec3.copy(agent.targetPosition, velocity);
    agent.targetState = AgentTargetState.VELOCITY;
    return true;
};
/**
 * Reset the move target for an agent.
 * @param crowd the crowd
 * @param agentId the ID of the agent
 * @returns true if the move target was reset, false otherwise
 */
const resetMoveTarget = (crowd, agentId) => {
    const agent = crowd.agents[agentId];
    if (!agent)
        return false;
    agent.targetRef = null;
    vec3.set(agent.targetPosition, 0, 0, 0);
    vec3.set(agent.desiredVelocity, 0, 0, 0);
    agent.targetReplan = false;
    agent.targetState = AgentTargetState.NONE;
    agent.targetPathIsPartial = false;
    return true;
};
const CHECK_LOOKAHEAD = 10;
const TARGET_REPLAN_DELAY_SECONDS = 1.0;
const _checkPathValidityNearestPolyResult = createFindNearestPolyResult();
const checkPathValidity = (crowd, navMesh, deltaTime) => {
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING)
            continue;
        agent.targetPathfindingTime += deltaTime;
        let replan = false;
        // first check that the current location is valid
        const agentNodeRef = agent.corridor.path[0];
        if (!isValidNodeRef(navMesh, agentNodeRef)) {
            // current location is invalid, try to reposition
            console.warn(`Agent ${agentId} has invalid current node ref, repositioning`, {
                agentNodeRef,
                corridorPath: agent.corridor.path,
                corridorPosition: agent.corridor.position,
                agentPosition: agent.position,
                velocity: agent.velocity,
                state: agent.state,
                targetState: agent.targetState,
            });
            const nearestPolyResult = findNearestPoly(_checkPathValidityNearestPolyResult, navMesh, agent.position, crowd.agentPlacementHalfExtents, agent.queryFilter);
            if (!nearestPolyResult.success) {
                // could not find location in navmesh, set agent state to invalid
                console.error(`Agent ${agentId} could not find valid position on navmesh, marking as INVALID`);
                agent.state = AgentState.INVALID;
                reset(agent.corridor, INVALID_NODE_REF, agent.position);
                resetLocalBoundary(agent.boundary);
                continue;
            }
            // Fix: Use nearestPolyResult.position (which is ON the navmesh) instead of agent.position
            fixPathStart(agent.corridor, nearestPolyResult.nodeRef, nearestPolyResult.position);
            resetLocalBoundary(agent.boundary);
            vec3.copy(agent.position, nearestPolyResult.position);
            replan = true;
        }
        // if the agent doesn't have a move target, or is controlled by velocity, no need to recover the target or replan
        if (agent.targetState === AgentTargetState.NONE || agent.targetState === AgentTargetState.VELOCITY) {
            continue;
        }
        // try to recover move request position
        if (agent.targetState !== AgentTargetState.NONE && agent.targetState !== AgentTargetState.FAILED) {
            if (agent.targetRef === null ||
                !isValidNodeRef(navMesh, agent.targetRef) ||
                !agent.queryFilter.passFilter(agent.targetRef, navMesh)) {
                // current target is not valid, try to reposition
                const nearestPolyResult = findNearestPoly(_checkPathValidityNearestPolyResult, navMesh, agent.targetPosition, crowd.agentPlacementHalfExtents, agent.queryFilter);
                if (!nearestPolyResult.success) {
                    // could not find location in navmesh, set agent state to invalid
                    agent.targetState = AgentTargetState.NONE;
                    agent.targetRef = null;
                    reset(agent.corridor, INVALID_NODE_REF, agent.position);
                }
                else {
                    // target poly became invalid, update to nearest valid poly
                    agent.targetRef = nearestPolyResult.nodeRef;
                    vec3.copy(agent.targetPosition, nearestPolyResult.position);
                    replan = true;
                }
            }
        }
        // if nearby corridor is not valid, replan
        const corridorValid = corridorIsValid(agent.corridor, CHECK_LOOKAHEAD, navMesh, agent.queryFilter);
        if (!corridorValid) {
            replan = true;
        }
        // if the end of the path is near and it is not the requested location, replan
        if (agent.targetState === AgentTargetState.VALID) {
            if (agent.targetPathfindingTime > TARGET_REPLAN_DELAY_SECONDS &&
                agent.corridor.path.length < CHECK_LOOKAHEAD &&
                agent.corridor.path[agent.corridor.path.length - 1] !== agent.targetRef) {
                replan = true;
            }
        }
        // try to replan path to goal
        if (replan && agent.targetState !== AgentTargetState.NONE) {
            requestMoveTargetReplan(crowd, agentId, agent.targetRef, agent.targetPosition);
        }
    }
};
const updateMoveRequests = (crowd, navMesh, deltaTime) => {
    // first, update pathfinding time for all agents in WAITING_FOR_PATH state
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.targetState === AgentTargetState.WAITING_FOR_PATH) {
            agent.targetPathfindingTime += deltaTime;
        }
    }
    // collect all agents that need pathfinding processing
    const pathfindingAgents = [];
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state === AgentState.INVALID ||
            agent.targetState === AgentTargetState.NONE ||
            agent.targetState === AgentTargetState.VELOCITY ||
            agent.targetRef === null) {
            continue;
        }
        if (agent.targetState === AgentTargetState.REQUESTING) {
            // init the pathfinding query and state
            initSlicedFindNodePath(navMesh, agent.slicedQuery, agent.corridor.path[0], agent.targetRef, agent.position, agent.targetPosition, agent.queryFilter);
            // quick search
            updateSlicedFindNodePath(navMesh, agent.slicedQuery, crowd.quickSearchIterations);
            // finalize the partial path from quick search
            const partialResult = agent.targetReplan
                ? finalizeSlicedFindNodePathPartial(navMesh, agent.slicedQuery, agent.corridor.path)
                : finalizeSlicedFindNodePath(navMesh, agent.slicedQuery);
            const reqPath = partialResult.path;
            const reqPathCount = reqPath.length;
            let reqPos = vec3.clone(agent.targetPosition);
            // if we got a path from the quick search
            if (reqPathCount > 0) {
                // check if this is a partial path (didn't reach target)
                if (reqPath[reqPathCount - 1] !== agent.targetRef) {
                    // partial path - constrain target position inside the last polygon
                    const closestPointResult = createGetClosestPointOnPolyResult();
                    const closestPoint = getClosestPointOnPoly(closestPointResult, navMesh, reqPath[reqPathCount - 1], agent.targetPosition);
                    if (closestPoint.success) {
                        reqPos = closestPoint.position;
                    }
                    else {
                        // failed to constrain position, fall back to current position
                        reqPos = vec3.clone(agent.position);
                        reqPath[0] = agent.corridor.path[0];
                        reqPath.length = 1;
                    }
                }
            }
            else {
                // could not find any path, start the request from current location
                reqPos = vec3.clone(agent.position);
                reqPath[0] = agent.corridor.path[0];
                reqPath.length = 1;
            }
            // immediately set the corridor with the partial path
            setPath(agent.corridor, reqPos, reqPath);
            resetLocalBoundary(agent.boundary);
            agent.targetPathIsPartial = false;
            // check if we reached the target with the quick search
            if (reqPathCount > 0 && reqPath[reqPathCount - 1] === agent.targetRef) {
                // reached target - we're done!
                agent.targetState = AgentTargetState.VALID;
                agent.targetPathfindingTime = 0;
            }
            else {
                // partial path - queue for full pathfinding
                agent.targetState = AgentTargetState.WAITING_FOR_QUEUE;
            }
        }
    }
    // process agents in WAITING_FOR_QUEUE - transition all waiting agents to full pathfinding
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.targetState === AgentTargetState.WAITING_FOR_QUEUE) {
            // initialize the full pathfinding query once when entering WAITING_FOR_PATH
            // start from the last polygon in the corridor (where partial path ended)
            const corridorPath = agent.corridor.path;
            const startRef = corridorPath[corridorPath.length - 1];
            initSlicedFindNodePath(navMesh, agent.slicedQuery, startRef, agent.targetRef, agent.corridor.target, agent.targetPosition, agent.queryFilter);
            agent.targetState = AgentTargetState.WAITING_FOR_PATH;
            pathfindingAgents.push(agentId);
        }
        else if (agent.targetState === AgentTargetState.WAITING_FOR_PATH) {
            pathfindingAgents.push(agentId);
        }
    }
    // sort agents by targetReplanTime (longest waiting gets priority)
    pathfindingAgents.sort((a, b) => crowd.agents[b].targetPathfindingTime - crowd.agents[a].targetPathfindingTime);
    // distribute global iteration budget across prioritized agents
    let remainingIterations = crowd.maxIterationsPerUpdate;
    for (const agentId of pathfindingAgents) {
        const agent = crowd.agents[agentId];
        if ((agent.slicedQuery.status & SlicedFindNodePathStatusFlags.IN_PROGRESS) !== 0 && remainingIterations > 0) {
            // allocate iterations for this agent (minimum 1, maximum remaining)
            const iterationsForAgent = Math.min(crowd.maxIterationsPerAgent, remainingIterations);
            const iterationsPerformed = updateSlicedFindNodePath(navMesh, agent.slicedQuery, iterationsForAgent);
            remainingIterations -= iterationsPerformed;
        }
        if ((agent.slicedQuery.status & SlicedFindNodePathStatusFlags.FAILURE) !== 0) {
            // pathfinding failed
            agent.targetState = AgentTargetState.FAILED;
            agent.targetPathfindingTime = 0;
            agent.targetPathIsPartial = false;
        }
        else if ((agent.slicedQuery.status & SlicedFindNodePathStatusFlags.SUCCESS) !== 0) {
            // pathfinding succeeded - now we need to merge the result with the current corridor
            // check if this is a partial path (best effort)
            agent.targetPathIsPartial = (agent.slicedQuery.status & SlicedFindNodePathStatusFlags.PARTIAL_RESULT) !== 0;
            const result = finalizeSlicedFindNodePath(navMesh, agent.slicedQuery);
            const newPath = result.path;
            const currentPath = agent.corridor.path;
            const currentPathCount = currentPath.length;
            let valid = true;
            let targetPos = vec3.clone(agent.targetPosition);
            // the agent might have moved whilst the pathfinding request was being processed,
            // so the path may have changed. We assume that the end of the current path is at
            // the same location where the pathfinding request was issued.
            // the last polygon in the current corridor should be the same as the first polygon
            // in the new pathfinding result.
            if (currentPathCount > 0 && newPath.length > 0) {
                if (currentPath[currentPathCount - 1] !== newPath[0]) {
                    valid = false;
                }
            }
            if (valid) {
                let mergedPath = [];
                // merge the current corridor path with the new pathfinding result
                if (currentPathCount > 1) {
                    // copy the current path (excluding the last polygon, which is the start of the new path)
                    mergedPath = currentPath.slice(0, currentPathCount - 1);
                    // append the new path
                    mergedPath.push(...newPath);
                    // remove trackbacks (A -> B -> A sequences)
                    for (let j = 0; j < mergedPath.length; j++) {
                        if (j - 1 >= 0 && j + 1 < mergedPath.length) {
                            if (mergedPath[j - 1] === mergedPath[j + 1]) {
                                // found a trackback: remove the middle and next element
                                mergedPath.splice(j - 1, 2);
                                j -= 2;
                            }
                        }
                    }
                }
                else {
                    // current path is just the start polygon, use the new path directly
                    mergedPath = [...newPath];
                }
                // check if this is a partial path - constrain target position if needed
                if (mergedPath.length > 0 && mergedPath[mergedPath.length - 1] !== agent.targetRef) {
                    // partial path - constrain target position to the last polygon
                    const lastPoly = mergedPath[mergedPath.length - 1];
                    const closestPointResult = createGetClosestPointOnPolyResult();
                    const closestPoint = getClosestPointOnPoly(closestPointResult, navMesh, lastPoly, agent.targetPosition);
                    if (closestPoint.success) {
                        targetPos = closestPoint.position;
                    }
                    else {
                        valid = false;
                    }
                }
                if (valid) {
                    setPath(agent.corridor, targetPos, mergedPath);
                    resetLocalBoundary(agent.boundary);
                    agent.targetState = AgentTargetState.VALID;
                    agent.targetPathfindingTime = 0; // Reset on success
                }
                else {
                    // something went wrong with constraining the target position
                    // retry if target is still valid
                    if (agent.targetRef !== null) {
                        agent.targetState = AgentTargetState.REQUESTING;
                        // Keep targetPathfindingTime so agent maintains priority when retrying
                    }
                    else {
                        agent.targetState = AgentTargetState.FAILED;
                        agent.targetPathfindingTime = 0; // Reset on failure
                    }
                }
            }
            else {
                // paths don't connect - the agent has moved too far from where pathfinding started
                // retry pathfinding from the new position if target is still valid
                if (agent.targetRef !== null) {
                    agent.targetState = AgentTargetState.REQUESTING;
                    // keep targetPathfindingTime so agent maintains priority when retrying
                }
                else {
                    agent.targetState = AgentTargetState.FAILED;
                    agent.targetPathfindingTime = 0; // Reset on failure
                }
            }
        }
    }
};
const updateNeighbours = (crowd) => {
    // uniform grid spatial partitioning, rebuilt each frame
    // find bounds and determine max query range
    let minX = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxZ = -Infinity;
    let maxQueryRange = 0;
    const agentIds = [];
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        agent.neis.length = 0;
        if (agent.state !== AgentState.WALKING)
            continue;
        agentIds.push(agentId);
        const x = agent.position[0];
        const z = agent.position[2];
        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minZ = Math.min(minZ, z);
        maxZ = Math.max(maxZ, z);
        maxQueryRange = Math.max(maxQueryRange, agent.collisionQueryRange);
    }
    if (agentIds.length === 0)
        return;
    // grid cell size = max query range (each agent checks its cell + surrounding 8 cells)
    const cellSize = maxQueryRange;
    if (cellSize < 0.01)
        return; // safety check
    const gridWidth = Math.ceil((maxX - minX) / cellSize) + 1;
    const gridHeight = Math.ceil((maxZ - minZ) / cellSize) + 1;
    // build grid - flat array with direct indexing (faster than Map)
    const gridSize = gridWidth * gridHeight;
    const grid = new Array(gridSize);
    // insert agents into grid
    for (const agentId of agentIds) {
        const agent = crowd.agents[agentId];
        const x = agent.position[0];
        const z = agent.position[2];
        const ix = Math.floor((x - minX) / cellSize);
        const iz = Math.floor((z - minZ) / cellSize);
        const key = iz * gridWidth + ix;
        let cell = grid[key];
        if (!cell) {
            cell = [];
            grid[key] = cell;
        }
        cell.push(agentId);
    }
    // query neighbors using grid
    for (const agentId of agentIds) {
        const agent = crowd.agents[agentId];
        const queryRangeSqr = agent.collisionQueryRange * agent.collisionQueryRange;
        const x = agent.position[0];
        const z = agent.position[2];
        const ix = Math.floor((x - minX) / cellSize);
        const iz = Math.floor((z - minZ) / cellSize);
        // check 3x3 grid around agent (including own cell)
        for (let dz = -1; dz <= 1; dz++) {
            for (let dx = -1; dx <= 1; dx++) {
                const checkZ = iz + dz;
                const checkX = ix + dx;
                if (checkX < 0 || checkX >= gridWidth || checkZ < 0 || checkZ >= gridHeight)
                    continue;
                const cellKey = checkZ * gridWidth + checkX;
                const cell = grid[cellKey];
                if (!cell)
                    continue;
                for (const otherAgentId of cell) {
                    if (otherAgentId === agentId)
                        continue;
                    const other = crowd.agents[otherAgentId];
                    const dx = agent.position[0] - other.position[0];
                    const dy = agent.position[1] - other.position[1];
                    const dz = agent.position[2] - other.position[2];
                    const distSqr = dx * dx + dy * dy + dz * dz;
                    if (distSqr < queryRangeSqr) {
                        agent.neis.push({ agentId: otherAgentId, dist: distSqr });
                    }
                }
            }
        }
    }
};
const updateLocalBoundaries = (crowd, navMesh) => {
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING || agent.corridor.path.length === 0) {
            continue;
        }
        // update boundary if agent has moved significantly or if boundary is invalid
        const updateThreshold = agent.collisionQueryRange * 0.25;
        const movedDistance = vec3.distance(agent.position, agent.boundary.center);
        if (movedDistance > updateThreshold ||
            !isLocalBoundaryValid(agent.boundary, navMesh, agent.queryFilter)) {
            updateLocalBoundary(agent.boundary, agent.corridor.path[0], agent.position, agent.collisionQueryRange, navMesh, agent.queryFilter);
        }
    }
};
const updateCorners = (crowd, navMesh) => {
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING ||
            agent.targetState === AgentTargetState.NONE ||
            agent.targetState === AgentTargetState.VELOCITY) {
            vec3.set(agent.desiredVelocity, 0, 0, 0);
            continue;
        }
        if (agent.state !== AgentState.WALKING || agent.targetState !== AgentTargetState.VALID) {
            vec3.set(agent.desiredVelocity, 0, 0, 0);
            continue;
        }
        // get corridor corners for steering
        const corners = findCorners(agent.corridor, navMesh, 3);
        if (!corners) {
            vec3.set(agent.desiredVelocity, 0, 0, 0);
            continue;
        }
        agent.corners = corners;
        // check to see if the corner after the next corner is directly visible, and short cut to there
        if ((agent.updateFlags & CrowdUpdateFlags.OPTIMIZE_VIS) !== 0 && agent.corners.length > 0) {
            const targetIndex = Math.min(1, agent.corners.length - 1);
            const target = agent.corners[targetIndex].position;
            optimizePathVisibility(agent.corridor, target, agent.pathOptimizationRange, navMesh, agent.queryFilter);
        }
    }
};
const dist2dSqr = (a, b) => {
    const dx = b[0] - a[0];
    const dz = b[2] - a[2];
    return dx * dx + dz * dz;
};
const agentIsOverOffMeshConnection = (agent, radius) => {
    if (agent.corners.length === 0)
        return false;
    const lastCorner = agent.corners[agent.corners.length - 1];
    if (lastCorner.type !== NodeType.OFFMESH)
        return false;
    const dist = dist2dSqr(agent.position, lastCorner.position);
    return dist < radius * radius;
};
const updateOffMeshConnectionTriggers = (crowd, navMesh) => {
    // trigger off mesh connections depending on next corners
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING ||
            agent.targetState === AgentTargetState.NONE ||
            agent.targetState === AgentTargetState.VELOCITY) {
            continue;
        }
        const triggerRadius = agent.radius * 2.25;
        if (agentIsOverOffMeshConnection(agent, triggerRadius)) {
            const offMeshConnectionNode = agent.corners[agent.corners.length - 1].nodeRef;
            if (!offMeshConnectionNode)
                continue;
            const result = moveOverOffMeshConnection(agent.corridor, offMeshConnectionNode, navMesh);
            if (result === false)
                continue;
            agent.state = AgentState.OFFMESH;
            // if autoTraverseOffMeshConnections is true, set up automatic animation
            // otherwise, still populate the data but the user must call completeOffMeshConnection manually
            agent.offMeshAnimation = {
                t: 0,
                duration: agent.autoTraverseOffMeshConnections ? 0.5 : -1,
                startPosition: vec3.clone(agent.position),
                endPosition: vec3.clone(result.endPosition),
                nodeRef: result.offMeshNodeRef,
            };
        }
    }
};
/**
 * Manually completes an off-mesh connection for an agent.
 * This should be called after custom off-mesh animation is complete.
 * @param crowd the crowd
 * @param agentId the agent id
 * @returns true if the off-mesh connection was completed successfully, false otherwise
 */
const completeOffMeshConnection = (crowd, agentId) => {
    const agent = crowd.agents[agentId];
    if (!agent)
        return false;
    if (agent.state !== AgentState.OFFMESH)
        return false;
    if (!agent.offMeshAnimation)
        return false;
    vec3.copy(agent.position, agent.offMeshAnimation.endPosition);
    // update velocity - set to zero during off-mesh connection
    vec3.set(agent.velocity, 0, 0, 0);
    vec3.set(agent.desiredVelocity, 0, 0, 0);
    // finish animation
    agent.offMeshAnimation = null;
    // prepare agent for walking
    agent.state = AgentState.WALKING;
    return true;
};
const _calcStraightSteerDirection_direction = vec3.create();
/**
 * Calculate straight steering direction (no anticipation).
 * Steers directly toward the first corner.
 */
const calcStraightSteerDirection = (agent, corners) => {
    if (corners.length === 0) {
        vec3.set(agent.desiredVelocity, 0, 0, 0);
        return;
    }
    const direction = vec3.subtract(_calcStraightSteerDirection_direction, corners[0].position, agent.position);
    direction[1] = 0; // Keep movement on XZ plane
    vec3.normalize(direction, direction);
    const speed = agent.maxSpeed;
    vec3.scale(agent.desiredVelocity, direction, speed);
};
const _calcSmoothSteerDirection_dir0 = vec3.create();
const _calcSmoothSteerDirection_dir1 = vec3.create();
const _calcSmoothSteerDirection_direction = vec3.create();
/**
 * Calculate smooth steering direction (with anticipation).
 * Blends between first and second corner for smoother turns.
 */
const calcSmoothSteerDirection = (agent, corners) => {
    if (corners.length === 0) {
        vec3.set(agent.desiredVelocity, 0, 0, 0);
        return;
    }
    const ip0 = 0;
    const ip1 = Math.min(1, corners.length - 1);
    const p0 = corners[ip0].position;
    const p1 = corners[ip1].position;
    const dir0 = vec3.subtract(_calcSmoothSteerDirection_dir0, p0, agent.position);
    const dir1 = vec3.subtract(_calcSmoothSteerDirection_dir1, p1, agent.position);
    dir0[1] = 0;
    dir1[1] = 0;
    const len0 = vec3.length(dir0);
    const len1 = vec3.length(dir1);
    if (len1 > 0.001) {
        vec3.scale(dir1, dir1, 1.0 / len1);
    }
    const direction = _calcSmoothSteerDirection_direction;
    direction[0] = dir0[0] - dir1[0] * len0 * 0.5;
    direction[1] = 0;
    direction[2] = dir0[2] - dir1[2] * len0 * 0.5;
    vec3.normalize(direction, direction);
    const speed = agent.maxSpeed;
    vec3.scale(agent.desiredVelocity, direction, speed);
};
const _getDistanceToGoalStart = vec2.create();
const _getDistanceToGoalEnd = vec2.create();
const getDistanceToGoal = (agent, range) => {
    if (agent.corners.length === 0)
        return range;
    const endPosition = agent.corners[agent.corners.length - 1];
    const isEndOfPath = (endPosition.flags & StraightPathPointFlags.END) !== 0;
    if (!isEndOfPath)
        return range;
    vec2.set(_getDistanceToGoalStart, endPosition.position[0], endPosition.position[2]);
    vec2.set(_getDistanceToGoalEnd, agent.position[0], agent.position[2]);
    const dist = vec2.distance(_getDistanceToGoalStart, _getDistanceToGoalEnd);
    return Math.min(range, dist);
};
const _updateSteering_separationDisp = vec3.create();
const _updateSteering_separationDiff = vec3.create();
const updateSteering = (crowd) => {
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.targetState === AgentTargetState.VELOCITY) {
            vec3.copy(agent.desiredVelocity, agent.targetPosition);
            continue;
        }
        const anticipateTurns = (agent.updateFlags & CrowdUpdateFlags.ANTICIPATE_TURNS) !== 0;
        // calculate steering direction
        if (anticipateTurns) {
            calcSmoothSteerDirection(agent, agent.corners);
        }
        else {
            calcStraightSteerDirection(agent, agent.corners);
        }
        // calculate speed scale, handles slowdown at the end of the path
        const slowDownRadius = agent.radius * 2;
        const speedScale = getDistanceToGoal(agent, slowDownRadius) / slowDownRadius;
        agent.desiredSpeed = agent.maxSpeed;
        vec3.scale(agent.desiredVelocity, agent.desiredVelocity, speedScale);
        // separation
        if ((agent.updateFlags & CrowdUpdateFlags.SEPARATION) !== 0) {
            const separationDist = agent.collisionQueryRange;
            const invSeparationDist = 1.0 / separationDist;
            const separationWeight = agent.separationWeight;
            let w = 0;
            const disp = _updateSteering_separationDisp;
            vec3.set(disp, 0, 0, 0);
            for (let j = 0; j < agent.neis.length; j++) {
                const neiId = agent.neis[j].agentId;
                const nei = crowd.agents[neiId];
                if (!nei)
                    continue;
                const diff = vec3.subtract(_updateSteering_separationDiff, agent.position, nei.position);
                diff[1] = 0; // ignore Y axis
                const distSqr = vec3.squaredLength(diff);
                if (distSqr < 0.00001)
                    continue;
                if (distSqr > separationDist * separationDist)
                    continue;
                const dist = Math.sqrt(distSqr);
                const weight = separationWeight * (1.0 - dist * invSeparationDist * (dist * invSeparationDist));
                // disp += diff * (weight / dist)
                vec3.scaleAndAdd(disp, disp, diff, weight / dist);
                w += 1.0;
            }
            if (w > 0.0001) {
                // adjust desired velocity: dvel += disp * (1.0 / w)
                vec3.scaleAndAdd(agent.desiredVelocity, agent.desiredVelocity, disp, 1.0 / w);
                // clamp desired velocity to desired speed
                const speedSqr = vec3.squaredLength(agent.desiredVelocity);
                const desiredSqr = agent.desiredSpeed * agent.desiredSpeed;
                if (speedSqr > desiredSqr && speedSqr > 0) {
                    vec3.scale(agent.desiredVelocity, agent.desiredVelocity, Math.sqrt(desiredSqr / speedSqr));
                }
            }
        }
    }
};
const updateVelocityPlanning = (crowd) => {
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING)
            continue;
        if (agent.updateFlags & CrowdUpdateFlags.OBSTACLE_AVOIDANCE) {
            // reset obstacle query
            resetObstacleAvoidanceQuery(agent.obstacleAvoidanceQuery);
            // add neighboring agents as circular obstacles
            for (const neighbor of agent.neis) {
                const neighborAgent = crowd.agents[neighbor.agentId];
                addCircleObstacle(agent.obstacleAvoidanceQuery, neighborAgent.position, neighborAgent.radius, neighborAgent.velocity, neighborAgent.desiredVelocity);
            }
            // add boundary segments as obstacles
            for (const segment of agent.boundary.segments) {
                const s = segment.s;
                const p1 = [s[0], s[1], s[2]];
                const p2 = [s[3], s[4], s[5]];
                // only add segments that are in front of the agent
                const triArea = (agent.position[0] - p1[0]) * (p2[2] - p1[2]) - (agent.position[2] - p1[2]) * (p2[0] - p1[0]);
                if (triArea < 0.0) {
                    continue;
                }
                addSegmentObstacle(agent.obstacleAvoidanceQuery, p1, p2);
            }
            // sample safe velocity using adaptive sampling
            sampleVelocityAdaptive(agent.obstacleAvoidanceQuery, agent.position, agent.radius, agent.maxSpeed, agent.velocity, agent.desiredVelocity, agent.obstacleAvoidance, agent.newVelocity, agent.obstacleAvoidanceDebugData);
        }
        else {
            // not using obstacle avoidance, set newVelocity to desiredVelocity
            vec3.copy(agent.newVelocity, agent.desiredVelocity);
        }
    }
};
const _integrateDv = vec3.create();
const integrate = (crowd, deltaTime) => {
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING)
            continue;
        // fake dynamic constraint - limit acceleration
        const maxDelta = agent.maxAcceleration * deltaTime;
        const dv = vec3.subtract(_integrateDv, agent.newVelocity, agent.velocity);
        const ds = vec3.length(dv);
        if (ds > maxDelta) {
            vec3.scale(dv, dv, maxDelta / ds);
        }
        vec3.add(agent.velocity, agent.velocity, dv);
        // integrate position
        if (vec3.length(agent.velocity) > 0.0001) {
            vec3.scaleAndAdd(agent.position, agent.position, agent.velocity, deltaTime);
        }
        else {
            vec3.set(agent.velocity, 0, 0, 0);
        }
    }
};
const _handleCollisions_diff = vec3.create();
const handleCollisions = (crowd) => {
    const COLLISION_RESOLVE_FACTOR = 0.7;
    // get all agents as an array for easier iteration
    const agentIds = Object.keys(crowd.agents);
    const agents = agentIds.map((id) => crowd.agents[id]);
    for (let iter = 0; iter < 4; iter++) {
        // first pass: calculate displacement for each agent
        for (let i = 0; i < agents.length; i++) {
            const agent = agents[i];
            if (agent.state !== AgentState.WALKING) {
                continue;
            }
            vec3.set(agent.displacement, 0, 0, 0);
            let w = 0;
            for (let j = 0; j < agent.neis.length; j++) {
                const neiAgentId = agent.neis[j].agentId;
                const nei = crowd.agents[neiAgentId];
                if (!nei)
                    continue;
                const diff = vec3.subtract(_handleCollisions_diff, agent.position, nei.position);
                diff[1] = 0; // ignore Y axis
                const distSqr = vec3.squaredLength(diff);
                const combinedRadius = agent.radius + nei.radius;
                if (distSqr > combinedRadius * combinedRadius) {
                    continue;
                }
                const dist = Math.sqrt(distSqr);
                let pen = combinedRadius - dist;
                if (dist < 0.0001) {
                    // agents on top of each other, try to choose diverging separation directions
                    const idx0 = i;
                    const idx1 = agentIds.indexOf(neiAgentId);
                    if (idx0 > idx1) {
                        vec3.set(diff, -agent.desiredVelocity[2], 0, agent.desiredVelocity[0]);
                    }
                    else {
                        vec3.set(diff, agent.desiredVelocity[2], 0, -agent.desiredVelocity[0]);
                    }
                    pen = 0.01;
                }
                else {
                    pen = (1.0 / dist) * (pen * 0.5) * COLLISION_RESOLVE_FACTOR;
                }
                vec3.scaleAndAdd(agent.displacement, agent.displacement, diff, pen);
                w += 1.0;
            }
            if (w > 0.0001) {
                const iw = 1.0 / w;
                vec3.scale(agent.displacement, agent.displacement, iw);
            }
        }
        // second pass: apply displacement to all agents
        for (let i = 0; i < agents.length; i++) {
            const agent = agents[i];
            if (agent.state !== AgentState.WALKING) {
                continue;
            }
            vec3.add(agent.position, agent.position, agent.displacement);
        }
    }
};
const updateCorridors = (crowd, navMesh) => {
    // update corridors for each agent
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING)
            continue;
        // move along navmesh
        movePosition(agent.corridor, agent.position, navMesh, agent.queryFilter);
        // get valid constrained position back
        vec3.copy(agent.position, agent.corridor.position);
        // if not using path, truncate the corridor to one poly
        if (agent.targetState === AgentTargetState.NONE || agent.targetState === AgentTargetState.VELOCITY) {
            reset(agent.corridor, agent.corridor.path[0], agent.position);
        }
    }
};
const offMeshConnectionUpdate = (crowd, deltaTime) => {
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (!agent.offMeshAnimation) {
            continue;
        }
        // only auto-update if autoTraverseOffMeshConnections is enabled
        // otherwise, the user is responsible for animation and calling completeOffMeshConnection
        if (!agent.autoTraverseOffMeshConnections) {
            continue;
        }
        const anim = agent.offMeshAnimation;
        // progress animation time
        anim.t += deltaTime;
        if (anim.t >= anim.duration) {
            // finish animation
            agent.offMeshAnimation = null;
            // prepare agent for walking
            agent.state = AgentState.WALKING;
            continue;
        }
        // update position
        const progress = anim.t / anim.duration;
        vec3.lerp(agent.position, anim.startPosition, anim.endPosition, progress);
        // update velocity - set to zero during off-mesh connection
        vec3.set(agent.velocity, 0, 0, 0);
        vec3.set(agent.desiredVelocity, 0, 0, 0);
    }
};
const updateTopologyOptimization = (crowd, navMesh, deltaTime) => {
    const OPT_TIME_THR = 0.5; // seconds
    const OPT_MAX_AGENTS = 1;
    const queue = [];
    for (const agentId in crowd.agents) {
        const agent = crowd.agents[agentId];
        if (agent.state !== AgentState.WALKING)
            continue;
        if (agent.targetState === AgentTargetState.NONE || agent.targetState === AgentTargetState.VELOCITY)
            continue;
        if ((agent.updateFlags & CrowdUpdateFlags.OPTIMIZE_TOPO) === 0)
            continue;
        agent.topologyOptTime += deltaTime;
        if (agent.topologyOptTime >= OPT_TIME_THR) {
            // Insert into queue based on greatest time (longest waiting gets priority)
            if (queue.length === 0) {
                queue.push({ agentId, time: agent.topologyOptTime });
            }
            else if (agent.topologyOptTime <= queue[queue.length - 1].time) {
                if (queue.length < OPT_MAX_AGENTS) {
                    queue.push({ agentId, time: agent.topologyOptTime });
                }
            }
            else {
                // Find insertion point (sorted by topologyOptTime descending)
                let insertIdx = 0;
                for (let i = 0; i < queue.length; i++) {
                    if (agent.topologyOptTime >= queue[i].time) {
                        insertIdx = i;
                        break;
                    }
                }
                queue.splice(insertIdx, 0, { agentId, time: agent.topologyOptTime });
                // Trim to max size
                if (queue.length > OPT_MAX_AGENTS) {
                    queue.length = OPT_MAX_AGENTS;
                }
            }
        }
    }
    for (const item of queue) {
        const agent = crowd.agents[item.agentId];
        optimizePathTopology(agent.corridor, navMesh, agent.queryFilter);
        agent.topologyOptTime = 0;
    }
};
/**
 * Update the crowd simulation.
 * @param crowd the crowd
 * @param navMesh the navigation mesh
 * @param deltaTime the time since the last update
 */
const update = (crowd, navMesh, deltaTime) => {
    // check whether agent paths are still valid
    checkPathValidity(crowd, navMesh, deltaTime);
    // optimize path topology for agents periodically
    updateTopologyOptimization(crowd, navMesh, deltaTime);
    // handle move requests since last update
    updateMoveRequests(crowd, navMesh, deltaTime);
    // update neighbour agents for each agent
    updateNeighbours(crowd);
    // update local boundary for each agent
    updateLocalBoundaries(crowd, navMesh);
    // update desired velocity based on steering to corners or velocity target
    updateCorners(crowd, navMesh);
    // trigger off mesh connections depending on next corners
    updateOffMeshConnectionTriggers(crowd, navMesh);
    // calculate steering
    updateSteering(crowd);
    // obstacle avoidance with other agents and local boundary
    updateVelocityPlanning(crowd);
    // integrate
    integrate(crowd, deltaTime);
    // handle agent x agent collisions
    handleCollisions(crowd);
    // update corridors
    updateCorridors(crowd, navMesh);
    // off mesh connection agent animations
    offMeshConnectionUpdate(crowd, deltaTime);
};
/**
 * Check if an agent is at or near the end of their corridor.
 * Works for both complete and partial paths - if the path is partial,
 * this checks if the agent reached the best-effort position.
 *
 * @param crowd the crowd
 * @param agentId the agent id
 * @param threshold distance threshold to consider "at target"
 * @returns true if the agent is at the end of their path
 */
const isAgentAtTarget = (crowd, agentId, threshold) => {
    const agent = crowd.agents[agentId];
    if (!agent)
        return false;
    // must have a valid target
    if (agent.targetState !== AgentTargetState.VALID)
        return false;
    // check if we have corners and the last corner is marked as END
    if (agent.corners.length === 0)
        return false;
    const endPosition = agent.corners[agent.corners.length - 1];
    const isEndOfPath = (endPosition.flags & StraightPathPointFlags.END) !== 0;
    if (!isEndOfPath)
        return false;
    // check distance to the end point
    const arrivalThreshold = threshold ?? agent.radius;
    const dist = vec3.distance(agent.position, endPosition.position);
    return dist <= arrivalThreshold;
};

var crowd = /*#__PURE__*/Object.freeze({
    __proto__: null,
    get AgentState () { return AgentState; },
    get AgentTargetState () { return AgentTargetState; },
    get CrowdUpdateFlags () { return CrowdUpdateFlags; },
    DEFAULT_OBSTACLE_AVOIDANCE_PARAMS: DEFAULT_OBSTACLE_AVOIDANCE_PARAMS,
    addAgent: addAgent,
    completeOffMeshConnection: completeOffMeshConnection,
    create: create,
    isAgentAtTarget: isAgentAtTarget,
    removeAgent: removeAgent,
    requestMoveTarget: requestMoveTarget,
    requestMoveVelocity: requestMoveVelocity,
    resetMoveTarget: resetMoveTarget,
    update: update
});

function generateSoloNavMesh(input, options) {
    const ctx = BuildContext.create();
    BuildContext.start(ctx, 'navmesh generation');
    const { positions, indices } = input;
    /* 0. define generation parameters */
    const { cellSize, cellHeight, walkableRadiusVoxels, walkableRadiusWorld, walkableClimbVoxels, walkableClimbWorld, walkableHeightVoxels, walkableHeightWorld, walkableSlopeAngleDegrees, borderSize, minRegionArea, mergeRegionArea, maxSimplificationError, maxEdgeLength, maxVerticesPerPoly, detailSampleDistance, detailSampleMaxError, } = options;
    /* 1. input positions and indices are already provided */
    BuildContext.start(ctx, 'mark walkable triangles');
    /* 2. mark walkable triangles */
    const triAreaIds = new Uint8Array(indices.length / 3).fill(0);
    markWalkableTriangles(positions, indices, triAreaIds, walkableSlopeAngleDegrees);
    BuildContext.end(ctx, 'mark walkable triangles');
    /* 3. rasterize the triangles to a voxel heightfield */
    BuildContext.start(ctx, 'rasterize triangles');
    const bounds = calculateMeshBounds(box3.create(), positions, indices);
    const [heightfieldWidth, heightfieldHeight] = calculateGridSize(vec2.create(), bounds, cellSize);
    const heightfield = createHeightfield(heightfieldWidth, heightfieldHeight, bounds, cellSize, cellHeight);
    rasterizeTriangles(ctx, heightfield, positions, indices, triAreaIds, walkableClimbVoxels);
    BuildContext.end(ctx, 'rasterize triangles');
    /* 4. filter walkable surfaces */
    // Once all geoemtry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    BuildContext.start(ctx, 'filter walkable surfaces');
    filterLowHangingWalkableObstacles(heightfield, walkableClimbVoxels);
    filterLedgeSpans(heightfield, walkableHeightVoxels, walkableClimbVoxels);
    filterWalkableLowHeightSpans(heightfield, walkableHeightVoxels);
    BuildContext.end(ctx, 'filter walkable surfaces');
    /* 5. compact the heightfield */
    // Compact the heightfield so that it is faster to handle from now on.
    // This will result more cache coherent data as well as the neighbours
    // between walkable cells will be calculated.
    BuildContext.start(ctx, 'build compact heightfield');
    const compactHeightfield = buildCompactHeightfield(ctx, walkableHeightVoxels, walkableClimbVoxels, heightfield);
    BuildContext.end(ctx, 'build compact heightfield');
    /* 6. erode the walkable area by the agent radius / walkable radius */
    BuildContext.start(ctx, 'erode walkable area');
    erodeWalkableArea(walkableRadiusVoxels, compactHeightfield);
    BuildContext.end(ctx, 'erode walkable area');
    /* 7. prepare for region partitioning by calculating a distance field along the walkable surface */
    BuildContext.start(ctx, 'build compact heightfield distance field');
    buildDistanceField(compactHeightfield);
    BuildContext.end(ctx, 'build compact heightfield distance field');
    /* 8. partition the walkable surface into simple regions without holes */
    BuildContext.start(ctx, 'build compact heightfield regions');
    // Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
    // There are 3 partitioning methods, each with some pros and cons:
    // 1) Watershed partitioning
    //   - the classic Recast partitioning
    //   - creates the nicest tessellation
    //   - usually slowest
    //   - partitions the heightfield into nice regions without holes or overlaps
    //   - the are some corner cases where this method creates produces holes and overlaps
    //      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
    //      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
    //   * generally the best choice if you precompute the navmesh, use this if you have large open areas
    // 2) Monotone partitioning
    //   - fastest
    //   - partitions the heightfield into regions without holes and overlaps (guaranteed)
    //   - creates long thin polygons, which sometimes causes paths with detours
    //   * use this if you want fast navmesh generation
    // 3) Layer partitoining
    //   - quite fast
    //   - partitions the heighfield into non-overlapping regions
    //   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
    //   - produces better triangles than monotone partitioning
    //   - does not have the corner cases of watershed partitioning
    //   - can be slow and create a bit ugly tessellation (still better than monotone)
    //     if you have large open areas with small obstacles (not a problem if you use tiles)
    //   * good choice to use for tiled navmesh with medium and small sized tiles
    buildRegions(ctx, compactHeightfield, borderSize, minRegionArea, mergeRegionArea);
    // buildRegionsMonotone(compactHeightfield, borderSize, minRegionArea, mergeRegionArea);
    // buildLayerRegions(compactHeightfield, borderSize, minRegionArea);
    BuildContext.end(ctx, 'build compact heightfield regions');
    /* 9. trace and simplify region contours */
    BuildContext.start(ctx, 'trace and simplify region contours');
    const contourSet = buildContours(ctx, compactHeightfield, maxSimplificationError, maxEdgeLength, ContourBuildFlags.CONTOUR_TESS_WALL_EDGES);
    BuildContext.end(ctx, 'trace and simplify region contours');
    /* 10. build polygons mesh from contours */
    BuildContext.start(ctx, 'build polygons mesh from contours');
    const polyMesh = buildPolyMesh(ctx, contourSet, maxVerticesPerPoly);
    for (let polyIndex = 0; polyIndex < polyMesh.nPolys; polyIndex++) {
        if (polyMesh.areas[polyIndex] === WALKABLE_AREA) {
            polyMesh.areas[polyIndex] = 0;
        }
        if (polyMesh.areas[polyIndex] === 0) {
            polyMesh.flags[polyIndex] = 1;
        }
    }
    BuildContext.end(ctx, 'build polygons mesh from contours');
    /* 11. create detail mesh which allows to access approximate height on each polygon */
    BuildContext.start(ctx, 'build detail mesh from contours');
    const polyMeshDetail = buildPolyMeshDetail(ctx, polyMesh, compactHeightfield, detailSampleDistance, detailSampleMaxError);
    BuildContext.end(ctx, 'build detail mesh from contours');
    BuildContext.end(ctx, 'navmesh generation');
    /* store intermediates for debugging */
    const intermediates = {
        buildContext: ctx,
        input: {
            positions,
            indices,
        },
        triAreaIds,
        heightfield,
        compactHeightfield,
        contourSet,
        polyMesh,
        polyMeshDetail,
    };
    /* create a single tile nav mesh */
    const nav = createNavMesh();
    nav.tileWidth = polyMesh.bounds[1][0] - polyMesh.bounds[0][0];
    nav.tileHeight = polyMesh.bounds[1][2] - polyMesh.bounds[0][2];
    vec3.copy(nav.origin, polyMesh.bounds[0]);
    const tilePolys = polyMeshToTilePolys(polyMesh);
    const tileDetailMesh = polyMeshDetailToTileDetailMesh(tilePolys.polys, polyMeshDetail);
    const tileParams = {
        bounds: polyMesh.bounds,
        vertices: tilePolys.vertices,
        polys: tilePolys.polys,
        detailMeshes: tileDetailMesh.detailMeshes,
        detailVertices: tileDetailMesh.detailVertices,
        detailTriangles: tileDetailMesh.detailTriangles,
        tileX: 0,
        tileY: 0,
        tileLayer: 0,
        cellSize,
        cellHeight,
        walkableHeight: walkableHeightWorld,
        walkableRadius: walkableRadiusWorld,
        walkableClimb: walkableClimbWorld,
    };
    const tile = buildTile(tileParams);
    addTile(nav, tile);
    return {
        navMesh: nav,
        intermediates,
    };
}

const buildNavMeshTile = (ctx, positions, indices, tileBounds, cellSize, cellHeight, borderSize, walkableSlopeAngleDegrees, walkableClimbVoxels, walkableHeightVoxels, walkableRadiusVoxels, tileSizeVoxels, minRegionArea, mergeRegionArea, maxSimplificationError, maxEdgeLength, maxVerticesPerPoly, detailSampleDistance, detailSampleMaxError) => {
    // Expand the heightfield bounding box by border size to find the extents of geometry we need to build this tile.
    //
    // This is done in order to make sure that the navmesh tiles connect correctly at the borders,
    // and the obstacles close to the border work correctly with the dilation process.
    // No polygons (or contours) will be created on the border area.
    //
    // IMPORTANT!
    //
    //   :''''''''':
    //   : +-----+ :
    //   : |     | :
    //   : |     |<--- tile to build
    //   : |     | :
    //   : +-----+ :<-- geometry needed
    //   :.........:
    //
    // You should use this bounding box to query your input geometry.
    //
    // For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
    // you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8 neighbours,
    // or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
    /* 1. expand the tile bounds by the border size */
    const expandedTileBounds = box3.clone(tileBounds);
    expandedTileBounds[0][0] -= borderSize * cellSize;
    expandedTileBounds[0][2] -= borderSize * cellSize;
    expandedTileBounds[1][0] += borderSize * cellSize;
    expandedTileBounds[1][2] += borderSize * cellSize;
    /* 2. get triangles overlapping the tile bounds */
    const trianglesInBox = [];
    const triangle = triangle3.create();
    for (let i = 0; i < indices.length; i += 3) {
        const a = indices[i];
        const b = indices[i + 1];
        const c = indices[i + 2];
        vec3.fromBuffer(triangle[0], positions, a * 3);
        vec3.fromBuffer(triangle[1], positions, b * 3);
        vec3.fromBuffer(triangle[2], positions, c * 3);
        if (box3.intersectsTriangle3(expandedTileBounds, triangle)) {
            trianglesInBox.push(a, b, c);
        }
    }
    /* 3. mark walkable triangles */
    const triAreaIds = new Uint8Array(trianglesInBox.length / 3).fill(0);
    markWalkableTriangles(positions, trianglesInBox, triAreaIds, walkableSlopeAngleDegrees);
    /* 4. rasterize the triangles to a voxel heightfield */
    const heightfieldWidth = Math.floor(tileSizeVoxels + borderSize * 2);
    const heightfieldHeight = Math.floor(tileSizeVoxels + borderSize * 2);
    const heightfield = createHeightfield(heightfieldWidth, heightfieldHeight, expandedTileBounds, cellSize, cellHeight);
    rasterizeTriangles(ctx, heightfield, positions, trianglesInBox, triAreaIds, walkableClimbVoxels);
    /* 5. filter walkable surfaces */
    filterLowHangingWalkableObstacles(heightfield, walkableClimbVoxels);
    filterLedgeSpans(heightfield, walkableHeightVoxels, walkableClimbVoxels);
    filterWalkableLowHeightSpans(heightfield, walkableHeightVoxels);
    /* 6. build the compact heightfield */
    const compactHeightfield = buildCompactHeightfield(ctx, walkableHeightVoxels, walkableClimbVoxels, heightfield);
    /* 7. erode the walkable area by the agent radius / walkable radius */
    erodeWalkableArea(walkableRadiusVoxels, compactHeightfield);
    /* 8. prepare for region partitioning by calculating a distance field along the walkable surface */
    buildDistanceField(compactHeightfield);
    /* 9. partition the walkable surface into simple regions without holes */
    buildRegions(ctx, compactHeightfield, borderSize, minRegionArea, mergeRegionArea);
    /* 10. trace and simplify region contours */
    const contourSet = buildContours(ctx, compactHeightfield, maxSimplificationError, maxEdgeLength, ContourBuildFlags.CONTOUR_TESS_WALL_EDGES);
    /* 11. build polygons mesh from contours */
    const polyMesh = buildPolyMesh(ctx, contourSet, maxVerticesPerPoly);
    for (let polyIndex = 0; polyIndex < polyMesh.nPolys; polyIndex++) {
        if (polyMesh.areas[polyIndex] === WALKABLE_AREA) {
            polyMesh.areas[polyIndex] = 0;
        }
        if (polyMesh.areas[polyIndex] === 0) {
            polyMesh.flags[polyIndex] = 1;
        }
    }
    /* 12. create detail mesh which allows to access approximate height on each polygon */
    const polyMeshDetail = buildPolyMeshDetail(ctx, polyMesh, compactHeightfield, detailSampleDistance, detailSampleMaxError);
    return {
        triAreaIds,
        expandedTileBounds,
        heightfield,
        compactHeightfield,
        contourSet,
        polyMesh,
        polyMeshDetail,
    };
};
function generateTiledNavMesh(input, options) {
    console.time('navmesh generation');
    const { positions, indices } = input;
    /* 0. define generation parameters */
    const { cellSize, cellHeight, tileSizeVoxels, tileSizeWorld, walkableRadiusVoxels, walkableRadiusWorld, walkableClimbVoxels, walkableClimbWorld, walkableHeightVoxels, walkableHeightWorld, walkableSlopeAngleDegrees, borderSize, minRegionArea, mergeRegionArea, maxSimplificationError, maxEdgeLength, maxVerticesPerPoly, detailSampleDistance, detailSampleMaxError, } = options;
    const ctx = BuildContext.create();
    /* 1. calculate mesh bounds and create tiled nav mesh */
    const meshBounds = calculateMeshBounds(box3.create(), positions, indices);
    const gridSize = calculateGridSize(vec2.create(), meshBounds, cellSize);
    const nav = createNavMesh();
    nav.tileWidth = tileSizeWorld;
    nav.tileHeight = tileSizeWorld;
    nav.origin = meshBounds[0];
    /* 2. initialize intermediates for debugging */
    const intermediates = {
        buildContext: ctx,
        input: {
            positions,
            indices,
        },
        inputBounds: meshBounds,
        triAreaIds: [],
        heightfield: [],
        compactHeightfield: [],
        contourSet: [],
        polyMesh: [],
        polyMeshDetail: [],
    };
    /* 3. generate tiles */
    const nTilesX = Math.floor((gridSize[0] + tileSizeVoxels - 1) / tileSizeVoxels);
    const nTilesY = Math.floor((gridSize[1] + tileSizeVoxels - 1) / tileSizeVoxels);
    for (let tileX = 0; tileX < nTilesX; tileX++) {
        for (let tileY = 0; tileY < nTilesY; tileY++) {
            const tileBounds = [
                [meshBounds[0][0] + tileX * tileSizeWorld, meshBounds[0][1], meshBounds[0][2] + tileY * tileSizeWorld],
                [
                    meshBounds[0][0] + (tileX + 1) * tileSizeWorld,
                    meshBounds[1][1],
                    meshBounds[0][2] + (tileY + 1) * tileSizeWorld,
                ],
            ];
            const { triAreaIds, polyMesh, polyMeshDetail, heightfield, compactHeightfield, contourSet } = buildNavMeshTile(ctx, positions, indices, tileBounds, cellSize, cellHeight, borderSize, walkableSlopeAngleDegrees, walkableClimbVoxels, walkableHeightVoxels, walkableRadiusVoxels, tileSizeVoxels, minRegionArea, mergeRegionArea, maxSimplificationError, maxEdgeLength, maxVerticesPerPoly, detailSampleDistance, detailSampleMaxError);
            if (polyMesh.vertices.length === 0)
                continue;
            intermediates.triAreaIds.push(triAreaIds);
            intermediates.heightfield.push(heightfield);
            intermediates.compactHeightfield.push(compactHeightfield);
            intermediates.contourSet.push(contourSet);
            intermediates.polyMesh.push(polyMesh);
            intermediates.polyMeshDetail.push(polyMeshDetail);
            const tilePolys = polyMeshToTilePolys(polyMesh);
            const tileDetailMesh = polyMeshDetailToTileDetailMesh(tilePolys.polys, polyMeshDetail);
            const tileParams = {
                bounds: polyMesh.bounds,
                vertices: tilePolys.vertices,
                polys: tilePolys.polys,
                detailMeshes: tileDetailMesh.detailMeshes,
                detailVertices: tileDetailMesh.detailVertices,
                detailTriangles: tileDetailMesh.detailTriangles,
                tileX,
                tileY,
                tileLayer: 0,
                cellSize,
                cellHeight,
                walkableHeight: walkableHeightWorld,
                walkableRadius: walkableRadiusWorld,
                walkableClimb: walkableClimbWorld,
            };
            const tile = buildTile(tileParams);
            addTile(nav, tile);
        }
    }
    console.timeEnd('navmesh generation');
    return {
        navMesh: nav,
        intermediates,
    };
}

const mergePositionsAndIndices = (meshes) => {
    const mergedPositions = [];
    const mergedIndices = [];
    const positionToIndex = {};
    let indexCounter = 0;
    for (const { positions, indices } of meshes) {
        for (let i = 0; i < indices.length; i++) {
            const pt = indices[i] * 3;
            const x = positions[pt];
            const y = positions[pt + 1];
            const z = positions[pt + 2];
            const key = `${x}_${y}_${z}`;
            let idx = positionToIndex[key];
            if (!idx) {
                positionToIndex[key] = idx = indexCounter;
                mergedPositions.push(x, y, z);
                indexCounter++;
            }
            mergedIndices.push(idx);
        }
    }
    return [mergedPositions, mergedIndices];
};

const floodFillNavMesh = (navMesh, startNodeRefs) => {
    const visited = new Set();
    const queue = [];
    // initialize queue with all seed points
    for (const startRef of startNodeRefs) {
        queue.push(startRef);
    }
    // bfs from all starting polygons to find all reachable polygons
    while (queue.length > 0) {
        const currentNodeRef = queue.shift();
        if (visited.has(currentNodeRef))
            continue;
        // add to visited
        visited.add(currentNodeRef);
        // follow all links
        const nodeIndex = getNodeRefIndex(currentNodeRef);
        const node = navMesh.nodes[nodeIndex];
        for (const linkIndex of node.links) {
            const link = navMesh.links[linkIndex];
            if (visited.has(link.toNodeRef))
                continue;
            queue.push(link.toNodeRef);
        }
    }
    // return reached and unreached polygons
    const reachable = Array.from(visited);
    const unreachable = [];
    for (const tileId in navMesh.tiles) {
        const tile = navMesh.tiles[tileId];
        for (let polyIndex = 0; polyIndex < tile.polys.length; polyIndex++) {
            const node = getNodeByTileAndPoly(navMesh, tile, polyIndex);
            if (!visited.has(node.ref)) {
                unreachable.push(node.ref);
            }
        }
    }
    return { reachable, unreachable };
};

export { crowd, floodFillNavMesh, generateSoloNavMesh, generateTiledNavMesh, localBoundary, mergePositionsAndIndices, obstacleAvoidance, pathCorridor };
//# sourceMappingURL=blocks.js.map

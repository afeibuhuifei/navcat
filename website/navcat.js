import { vec3, vec2, box3, clamp, triangle2, circle } from 'mathcat';

var BuildContextLogType;
(function (BuildContextLogType) {
    BuildContextLogType[BuildContextLogType["INFO"] = 0] = "INFO";
    BuildContextLogType[BuildContextLogType["WARNING"] = 1] = "WARNING";
    BuildContextLogType[BuildContextLogType["ERROR"] = 2] = "ERROR";
})(BuildContextLogType || (BuildContextLogType = {}));
const create = () => {
    return {
        logs: [],
        times: [],
        _startTimes: {},
    };
};
const start = (context, name) => {
    context._startTimes[name] = performance.now();
};
const end = (context, name) => {
    const now = performance.now();
    const start = context._startTimes[name];
    const duration = now - start;
    delete context._startTimes[name];
    context.times.push({ name, duration });
};
const info = (context, message) => {
    context.logs.push({
        type: BuildContextLogType.INFO,
        message,
    });
};
const warn = (context, message) => {
    context.logs.push({
        type: BuildContextLogType.WARNING,
        message,
    });
};
const error = (context, message) => {
    context.logs.push({
        type: BuildContextLogType.ERROR,
        message,
    });
};
const BuildContext = {
    create,
    start,
    end,
    info,
    warn,
    error,
};

// Direction offsets for 4-directional neighbor access (N, E, S, W)
const DIR_OFFSETS = [
    // North (negative Z)
    [-1, 0],
    // East (positive X)
    [0, 1],
    // South (positive Z)
    [1, 0],
    // West (negative X)
    [0, -1],
];
const getDirOffsetX = (dir) => {
    return DIR_OFFSETS[dir & 0x03][0];
};
const getDirOffsetY = (dir) => {
    return DIR_OFFSETS[dir & 0x03][1];
};
const getDirForOffset = (x, y) => {
    for (let i = 0; i < DIR_OFFSETS.length; i++) {
        if (DIR_OFFSETS[i][0] === x && DIR_OFFSETS[i][1] === y) {
            return i;
        }
    }
    return 0; // Default to North if no match
};
const AXIS_X = 0;
const AXIS_Y = 1;
const AXIS_Z = 2;
const MULTIPLE_REGS = 0;
const MESH_NULL_IDX = -1;
const BORDER_VERTEX = 0x10000;
const CONTOUR_REG_MASK = 0xffff;
const AREA_BORDER = 0x20000;
const NULL_AREA = 0;
const WALKABLE_AREA = 1;
const BORDER_REG = 0x8000;
const NOT_CONNECTED = 0x3f; // 63
const MAX_HEIGHT = 0xffff;
const MAX_LAYERS = NOT_CONNECTED - 1;
/**
 * A flag that indicates that an entity links to an external enttity.
 * (E.g. A polygon edge is a portal that links to another polygon.)
 */
const POLY_NEIS_FLAG_EXT_LINK = 0x8000;
/**
 * A mask for the direction of an external link.
 */
const POLY_NEIS_FLAG_EXT_LINK_DIR_MASK = 0xff;

const EPS = 1e-6;
/**
 * Calculates the closest point on a line segment to a given point in 2D (XZ plane)
 * @param out Output parameter for the closest point
 * @param pt The point
 * @param p First endpoint of the segment
 * @param q Second endpoint of the segment
 */
const closestPtSeg2d = (out, pt, p, q) => {
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
    out[0] = p[0] + t * pqx;
    out[1] = p[1]; // keep original Y value from p
    out[2] = p[2] + t * pqz;
};
/**
 * Tests if a point is inside a polygon in 2D (XZ plane)
 */
const pointInPoly = (point, vertices, nVertices) => {
    let inside = false;
    const x = point[0];
    const z = point[2];
    for (let l = nVertices, i = 0, j = l - 1; i < l; j = i++) {
        const xj = vertices[j * 3], zj = vertices[j * 3 + 2], xi = vertices[i * 3], zi = vertices[i * 3 + 2];
        const where = (zi - zj) * (x - xi) - (xi - xj) * (z - zi);
        if (zj < zi) {
            if (z >= zj && z < zi) {
                if (where === 0) {
                    // point on the line
                    return true;
                }
                if (where > 0) {
                    if (z === zj) {
                        // ray intersects vertex
                        if (z > vertices[(j === 0 ? l - 1 : j - 1) * 3 + 2]) {
                            inside = !inside;
                        }
                    }
                    else {
                        inside = !inside;
                    }
                }
            }
        }
        else if (zi < zj) {
            if (z > zi && z <= zj) {
                if (where === 0) {
                    // point on the line
                    return true;
                }
                if (where < 0) {
                    if (z === zj) {
                        // ray intersects vertex
                        if (z < vertices[(j === 0 ? l - 1 : j - 1) * 3 + 2]) {
                            inside = !inside;
                        }
                    }
                    else {
                        inside = !inside;
                    }
                }
            }
        }
        else if (z === zi && ((x >= xj && x <= xi) || (x >= xi && x <= xj))) {
            // point on horizontal edge
            return true;
        }
    }
    return inside;
};
const _distPtTriV0 = vec3.create();
const _distPtTriV1 = vec3.create();
const _distPtTriV2 = vec3.create();
const _distPtTriVec0 = vec2.create();
const _distPtTriVec1 = vec2.create();
const _distPtTriVec2 = vec2.create();
const distPtTri = (p, a, b, c) => {
    const v0 = _distPtTriV0;
    const v1 = _distPtTriV1;
    const v2 = _distPtTriV2;
    vec3.subtract(v0, c, a);
    vec3.subtract(v1, b, a);
    vec3.subtract(v2, p, a);
    _distPtTriVec0[0] = v0[0];
    _distPtTriVec0[1] = v0[2];
    _distPtTriVec1[0] = v1[0];
    _distPtTriVec1[1] = v1[2];
    _distPtTriVec2[0] = v2[0];
    _distPtTriVec2[1] = v2[2];
    const dot00 = vec2.dot(_distPtTriVec0, _distPtTriVec0);
    const dot01 = vec2.dot(_distPtTriVec0, _distPtTriVec1);
    const dot02 = vec2.dot(_distPtTriVec0, _distPtTriVec2);
    const dot11 = vec2.dot(_distPtTriVec1, _distPtTriVec1);
    const dot12 = vec2.dot(_distPtTriVec1, _distPtTriVec2);
    // Compute barycentric coordinates
    const invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    const u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    const v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    // If point lies inside the triangle, return interpolated y-coord.
    const EPS_TRI = 1e-4;
    if (u >= -EPS_TRI && v >= -EPS_TRI && u + v <= 1 + EPS_TRI) {
        const y = a[1] + v0[1] * u + v1[1] * v;
        return Math.abs(y - p[1]);
    }
    return Number.MAX_VALUE;
};
const _distPtSegP = vec3.create();
const _distPtSegQ = vec3.create();
const distancePtSeg$1 = (pt, p, q) => {
    const pq = _distPtSegP;
    const d_vec = _distPtSegQ;
    vec3.subtract(pq, q, p); // pq = q - p
    vec3.subtract(d_vec, pt, p); // d_vec = pt - p
    const d = vec3.dot(pq, pq);
    let t = vec3.dot(pq, d_vec);
    if (d > 0)
        t /= d;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;
    // calculate closest point on segment: p + t * pq
    vec3.scale(pq, pq, t);
    vec3.add(pq, p, pq);
    // calculate distance vector: closest_point - pt
    vec3.subtract(pq, pq, pt);
    return vec3.dot(pq, pq); // return squared distance
};
const createDistPtSeg2dResult = () => ({
    dist: 0,
    t: 0,
});
const distancePtSeg2d = (out, pt, p, q) => {
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
    const closeDx = p[0] + t * pqx - pt[0];
    const closeDz = p[2] + t * pqz - pt[2];
    const dist = closeDx * closeDx + closeDz * closeDz;
    out.dist = dist;
    out.t = t;
    return out;
};
const createDistancePtSegSqr2dResult = () => ({
    distSqr: 0,
    t: 0,
});
const distancePtSegSqr2d = (out, pt, p, q) => {
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
    const closestX = p[0] + t * pqx;
    const closestZ = p[2] + t * pqz;
    const distX = closestX - pt[0];
    const distZ = closestZ - pt[2];
    const distSqr = distX * distX + distZ * distZ;
    out.distSqr = distSqr;
    out.t = t;
    return out;
};
const _distPtTriA = vec3.create();
const _distPtTriB = vec3.create();
const _distPtTriC = vec3.create();
const distToTriMesh = (p, verts, tris, ntris) => {
    let dmin = Number.MAX_VALUE;
    for (let i = 0; i < ntris; ++i) {
        const va = tris[i * 4 + 0] * 3;
        const vb = tris[i * 4 + 1] * 3;
        const vc = tris[i * 4 + 2] * 3;
        vec3.fromBuffer(_distPtTriA, verts, va);
        vec3.fromBuffer(_distPtTriB, verts, vb);
        vec3.fromBuffer(_distPtTriC, verts, vc);
        const d = distPtTri(p, _distPtTriA, _distPtTriB, _distPtTriC);
        if (d < dmin)
            dmin = d;
    }
    if (dmin === Number.MAX_VALUE)
        return -1;
    return dmin;
};
const _distToPolyVj = vec3.create();
const _distToPolyVi = vec3.create();
const _distToPoly_distPtSeg2dResult = createDistPtSeg2dResult();
const distToPoly = (nvert, verts, p) => {
    let dmin = Number.MAX_VALUE;
    let c = 0;
    for (let i = 0, j = nvert - 1; i < nvert; j = i++) {
        const vi = i * 3;
        const vj = j * 3;
        if (verts[vi + 2] > p[2] !== verts[vj + 2] > p[2] &&
            p[0] < ((verts[vj] - verts[vi]) * (p[2] - verts[vi + 2])) / (verts[vj + 2] - verts[vi + 2]) + verts[vi]) {
            c = c === 0 ? 1 : 0;
        }
        vec3.fromBuffer(_distToPolyVj, verts, vj);
        vec3.fromBuffer(_distToPolyVi, verts, vi);
        distancePtSeg2d(_distToPoly_distPtSeg2dResult, p, _distToPolyVj, _distToPolyVi);
        dmin = Math.min(dmin, _distToPoly_distPtSeg2dResult.dist);
    }
    return c ? -dmin : dmin;
};
/**
 * Calculates the closest height point on a triangle using barycentric coordinates.
 * @param p The point to project
 * @param a First triangle vertex
 * @param b Second triangle vertex
 * @param c Third triangle vertex
 * @returns Height at position, or NaN if point is not inside triangle
 */
const closestHeightPointTriangle = (p, a, b, c) => {
    const EPS = 1e-6;
    const v0x = c[0] - a[0];
    const v0y = c[1] - a[1];
    const v0z = c[2] - a[2];
    const v1x = b[0] - a[0];
    const v1y = b[1] - a[1];
    const v1z = b[2] - a[2];
    const v2x = p[0] - a[0];
    const v2z = p[2] - a[2];
    // Compute scaled barycentric coordinates
    let denom = v0x * v1z - v0z * v1x;
    if (Math.abs(denom) < EPS) {
        return NaN;
    }
    let u = v1z * v2x - v1x * v2z;
    let v = v0x * v2z - v0z * v2x;
    if (denom < 0) {
        denom = -denom;
        u = -u;
        v = -v;
    }
    // If point lies inside the triangle, return interpolated ycoord.
    if (u >= 0.0 && v >= 0.0 && u + v <= denom) {
        return a[1] + (v0y * u + v1y * v) / denom;
    }
    return NaN;
};
const _overlapSegAB = vec2.create();
const _overlapSegAD = vec2.create();
const _overlapSegAC = vec2.create();
const _overlapSegCD = vec2.create();
const _overlapSegCA = vec2.create();
const overlapSegSeg2d = (a, b, c, d) => {
    // calculate cross products for line segment intersection test
    const ab = _overlapSegAB;
    const ad = _overlapSegAD;
    const ac = _overlapSegAC;
    vec2.subtract(ab, b, a); // b - a
    vec2.subtract(ad, d, a); // d - a
    const a1 = ab[0] * ad[1] - ab[1] * ad[0];
    vec2.subtract(ac, c, a); // c - a
    const a2 = ab[0] * ac[1] - ab[1] * ac[0];
    if (a1 * a2 < 0.0) {
        const cd = _overlapSegCD;
        const ca = _overlapSegCA;
        vec2.subtract(cd, d, c); // d - c
        vec2.subtract(ca, a, c); // a - c
        const a3 = cd[0] * ca[1] - cd[1] * ca[0];
        const a4 = a3 + a2 - a1;
        if (a3 * a4 < 0.0)
            return true;
    }
    return false;
};
/**
 * 2D signed area in XZ plane (positive if c is to the left of ab)
 */
const triArea2D = (a, b, c) => {
    const abx = b[0] - a[0];
    const abz = b[2] - a[2];
    const acx = c[0] - a[0];
    const acz = c[2] - a[2];
    return acx * abz - abx * acz;
};
const createIntersectSegSeg2DResult = () => ({
    hit: false,
    s: 0,
    t: 0,
});
/**
 * Segment-segment intersection in XZ plane.
 * Returns { hit, s, t } where
 *  P = a + s*(b-a) and Q = c + t*(d-c). Hit only if both s and t are within [0,1].
 */
const intersectSegSeg2D = (out, a, b, c, d) => {
    const bax = b[0] - a[0];
    const baz = b[2] - a[2];
    const dcx = d[0] - c[0];
    const dcz = d[2] - c[2];
    const acx = a[0] - c[0];
    const acz = a[2] - c[2];
    const denom = dcz * bax - dcx * baz;
    if (Math.abs(denom) < 1e-12) {
        out.hit = false;
        out.s = 0;
        out.t = 0;
        return out;
    }
    const s = (dcx * acz - dcz * acx) / denom;
    const t = (bax * acz - baz * acx) / denom;
    const hit = !(s < 0 || s > 1 || t < 0 || t > 1);
    out.hit = hit;
    out.s = s;
    out.t = t;
    return out;
};
const _polyMinExtentPt = vec3.create();
const _polyMinExtentP1 = vec3.create();
const _polyMinExtentP2 = vec3.create();
const _polyMinExtent_distPtSeg2dResult = createDistPtSeg2dResult();
// calculate minimum extend of the polygon.
const polyMinExtent = (verts, nverts) => {
    let minDist = Number.MAX_VALUE;
    for (let i = 0; i < nverts; i++) {
        const ni = (i + 1) % nverts;
        const p1 = i * 3;
        const p2 = ni * 3;
        let maxEdgeDist = 0;
        for (let j = 0; j < nverts; j++) {
            if (j === i || j === ni)
                continue;
            const ptIdx = j * 3;
            vec3.fromBuffer(_polyMinExtentPt, verts, ptIdx);
            vec3.fromBuffer(_polyMinExtentP1, verts, p1);
            vec3.fromBuffer(_polyMinExtentP2, verts, p2);
            distancePtSeg2d(_polyMinExtent_distPtSeg2dResult, _polyMinExtentPt, _polyMinExtentP1, _polyMinExtentP2);
            maxEdgeDist = Math.max(maxEdgeDist, _polyMinExtent_distPtSeg2dResult.dist);
        }
        minDist = Math.min(minDist, maxEdgeDist);
    }
    return Math.sqrt(minDist);
};
/**
 * Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
 * The vectors are projected onto the xz-plane, so the y-values are ignored.
 * @param u The LHV vector [(x, y, z)]
 * @param v The RHV vector [(x, y, z)]
 * @returns The perp dot product on the xz-plane.
 */
const vperp2D = (u, v) => {
    return u[2] * v[0] - u[0] * v[2];
};
const createIntersectSegmentPoly2DResult = () => ({
    intersects: false,
    tmin: 0,
    tmax: 0,
    segMin: -1,
    segMax: -1,
});
const _intersectSegmentPoly2DVi = vec3.create();
const _intersectSegmentPoly2DVj = vec3.create();
const _intersectSegmentPoly2DDir = vec3.create();
const _intersectSegmentPoly2DToStart = vec3.create();
const _intersectSegmentPoly2DEdge = vec3.create();
/**
 * Intersects a segment with a polygon in 2D (ignoring Y).
 * Uses the Sutherland-Hodgman clipping algorithm approach.
 *
 * @param result The result object to store intersection data
 * @param startPosition Start position of the segment
 * @param endPosition End position of the segment
 * @param verts Polygon vertices as flat array [x,y,z,x,y,z,...]
 * @param nv Number of vertices in the polygon
 */
const intersectSegmentPoly2D = (result, startPosition, endPosition, nv, verts) => {
    result.intersects = false;
    result.tmin = 0;
    result.tmax = 1;
    result.segMin = -1;
    result.segMax = -1;
    const dir = vec3.subtract(_intersectSegmentPoly2DDir, endPosition, startPosition);
    const vi = _intersectSegmentPoly2DVi;
    const vj = _intersectSegmentPoly2DVj;
    const edge = _intersectSegmentPoly2DEdge;
    const diff = _intersectSegmentPoly2DToStart;
    for (let i = 0, j = nv - 1; i < nv; j = i, i++) {
        vec3.fromBuffer(vi, verts, i * 3);
        vec3.fromBuffer(vj, verts, j * 3);
        vec3.subtract(edge, vi, vj);
        vec3.subtract(diff, startPosition, vj);
        const n = vperp2D(edge, diff);
        const d = vperp2D(dir, edge);
        if (Math.abs(d) < EPS) {
            // S is nearly parallel to this edge
            if (n < 0) {
                return result;
            }
            continue;
        }
        const t = n / d;
        if (d < 0) {
            // segment S is entering across this edge
            if (t > result.tmin) {
                result.tmin = t;
                result.segMin = j;
                // S enters after leaving polygon
                if (result.tmin > result.tmax) {
                    return result;
                }
            }
        }
        else {
            // segment S is leaving across this edge
            if (t < result.tmax) {
                result.tmax = t;
                result.segMax = j;
                // S leaves before entering polygon
                if (result.tmax < result.tmin) {
                    return result;
                }
            }
        }
    }
    result.intersects = true;
    return result;
};
const _randomPointInConvexPolyVa = vec3.create();
const _randomPointInConvexPolyVb = vec3.create();
const _randomPointInConvexPolyVc = vec3.create();
/**
 * Generates a random point inside a convex polygon using barycentric coordinates.
 *
 * @param verts - Polygon vertices as flat array [x,y,z,x,y,z,...]
 * @param areas - Temporary array for triangle areas (will be modified)
 * @param s - Random value [0,1] for triangle selection
 * @param t - Random value [0,1] for point within triangle
 * @param out - Output point [x,y,z]
 */
const randomPointInConvexPoly = (out, nv, verts, areas, s, t) => {
    // calculate cumulative triangle areas for weighted selection
    let areaSum = 0;
    for (let i = 2; i < nv; i++) {
        const va = [verts[0], verts[1], verts[2]];
        const vb = [verts[(i - 1) * 3], verts[(i - 1) * 3 + 1], verts[(i - 1) * 3 + 2]];
        const vc = [verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2]];
        areas[i] = triArea2D(va, vb, vc);
        areaSum += Math.max(0.001, areas[i]);
    }
    // choose triangle based on area-weighted random selection
    const thr = s * areaSum;
    let acc = 0;
    let tri = nv - 1;
    for (let i = 2; i < nv; i++) {
        acc += Math.max(0.001, areas[i]);
        if (thr <= acc) {
            tri = i;
            break;
        }
    }
    // generate random point in triangle using barycentric coordinates
    // standard method: use square root for uniform distribution
    let u = Math.sqrt(t);
    let v = 1 - t;
    // ensure the point is inside the triangle
    if (u + v > 1) {
        u = 1 - u;
        v = 1 - v;
    }
    const w = 1 - u - v;
    const va = vec3.fromBuffer(_randomPointInConvexPolyVa, verts, 0);
    const vb = vec3.fromBuffer(_randomPointInConvexPolyVb, verts, (tri - 1) * 3);
    const vc = vec3.fromBuffer(_randomPointInConvexPolyVc, verts, tri * 3);
    out[0] = u * va[0] + v * vb[0] + w * vc[0];
    out[1] = u * va[1] + v * vb[1] + w * vc[1];
    out[2] = u * va[2] + v * vb[2] + w * vc[2];
    return out;
};
/**
 * Projects a polygon onto an axis and returns the min/max projection values.
 * @param out Output tuple [min, max]
 * @param axis The axis to project onto [x, z]
 * @param verts Polygon vertices [x,y,z,x,y,z,...]
 * @param nverts Number of vertices
 */
const projectPoly = (out, axis, verts, nverts) => {
    let min = axis[0] * verts[0] + axis[1] * verts[2]; // dot product with first vertex (x,z)
    let max = min;
    for (let i = 1; i < nverts; i++) {
        const dot = axis[0] * verts[i * 3] + axis[1] * verts[i * 3 + 2]; // dot product (x,z)
        min = Math.min(min, dot);
        max = Math.max(max, dot);
    }
    out[0] = min;
    out[1] = max;
};
/**
 * Checks if two ranges overlap with epsilon tolerance.
 * @param amin Min value of range A
 * @param amax Max value of range A
 * @param bmin Min value of range B
 * @param bmax Max value of range B
 * @param eps Epsilon tolerance
 * @returns True if ranges overlap
 */
const overlapRange = (amin, amax, bmin, bmax, eps) => {
    return !(amin + eps > bmax || amax - eps < bmin);
};
const _overlapPolyPolyNormal = vec2.create();
const _overlapPolyPolyVa = vec2.create();
const _overlapPolyPolyVb = vec2.create();
const _overlapPolyPolyProjA = [0, 0];
const _overlapPolyPolyProjB = [0, 0];
/**
 * Tests if two convex polygons overlap in 2D (XZ plane).
 * Uses the separating axis theorem - matches the C++ dtOverlapPolyPoly2D implementation.
 * All vertices are projected onto the xz-plane, so the y-values are ignored.
 *
 * @param vertsA Vertices of the first polygon [x,y,z,x,y,z,...]
 * @param nvertsA Number of vertices in the first polygon
 * @param vertsB Vertices of the second polygon [x,y,z,x,y,z,...]
 * @param nvertsB Number of vertices in the second polygon
 * @returns True if the polygons overlap
 */
const overlapPolyPoly2D = (vertsA, nvertsA, vertsB, nvertsB) => {
    const eps = 1e-4;
    // Check separation along each edge normal of polygon A
    for (let i = 0, j = nvertsA - 1; i < nvertsA; j = i++) {
        const va = _overlapPolyPolyVa;
        const vb = _overlapPolyPolyVb;
        va[0] = vertsA[j * 3]; // x
        va[1] = vertsA[j * 3 + 2]; // z
        vb[0] = vertsA[i * 3]; // x
        vb[1] = vertsA[i * 3 + 2]; // z
        // Calculate edge normal: n = { vb[z]-va[z], -(vb[x]-va[x]) }
        const normal = _overlapPolyPolyNormal;
        normal[0] = vb[1] - va[1]; // z component
        normal[1] = -(vb[0] - va[0]); // negative x component
        // Project both polygons onto this normal
        const projA = _overlapPolyPolyProjA;
        const projB = _overlapPolyPolyProjB;
        projectPoly(projA, normal, vertsA, nvertsA);
        projectPoly(projB, normal, vertsB, nvertsB);
        // Check if projections are separated
        if (!overlapRange(projA[0], projA[1], projB[0], projB[1], eps)) {
            // Found separating axis
            return false;
        }
    }
    // Check separation along each edge normal of polygon B
    for (let i = 0, j = nvertsB - 1; i < nvertsB; j = i++) {
        const va = _overlapPolyPolyVa;
        const vb = _overlapPolyPolyVb;
        va[0] = vertsB[j * 3]; // x
        va[1] = vertsB[j * 3 + 2]; // z
        vb[0] = vertsB[i * 3]; // x
        vb[1] = vertsB[i * 3 + 2]; // z
        // Calculate edge normal: n = { vb[z]-va[z], -(vb[x]-va[x]) }
        const normal = _overlapPolyPolyNormal;
        normal[0] = vb[1] - va[1]; // z component
        normal[1] = -(vb[0] - va[0]); // negative x component
        // Project both polygons onto this normal
        const projA = _overlapPolyPolyProjA;
        const projB = _overlapPolyPolyProjB;
        projectPoly(projA, normal, vertsA, nvertsA);
        projectPoly(projB, normal, vertsB, nvertsB);
        // Check if projections are separated
        if (!overlapRange(projA[0], projA[1], projB[0], projB[1], eps)) {
            // Found separating axis
            return false;
        }
    }
    // No separating axis found, polygons overlap
    return true;
};

var geometry = /*#__PURE__*/Object.freeze({
    __proto__: null,
    closestHeightPointTriangle: closestHeightPointTriangle,
    closestPtSeg2d: closestPtSeg2d,
    createDistPtSeg2dResult: createDistPtSeg2dResult,
    createDistancePtSegSqr2dResult: createDistancePtSegSqr2dResult,
    createIntersectSegSeg2DResult: createIntersectSegSeg2DResult,
    createIntersectSegmentPoly2DResult: createIntersectSegmentPoly2DResult,
    distPtTri: distPtTri,
    distToPoly: distToPoly,
    distToTriMesh: distToTriMesh,
    distancePtSeg: distancePtSeg$1,
    distancePtSeg2d: distancePtSeg2d,
    distancePtSegSqr2d: distancePtSegSqr2d,
    intersectSegSeg2D: intersectSegSeg2D,
    intersectSegmentPoly2D: intersectSegmentPoly2D,
    overlapPolyPoly2D: overlapPolyPoly2D,
    overlapSegSeg2d: overlapSegSeg2d,
    pointInPoly: pointInPoly,
    polyMinExtent: polyMinExtent,
    randomPointInConvexPoly: randomPointInConvexPoly,
    triArea2D: triArea2D
});

/**
 * Helper function to set connection data in a span
 */
const setCon = (span, dir, layerIndex) => {
    const shift = dir * 6; // 6 bits per direction
    const mask = 0x3f << shift; // 6-bit mask
    span.con = (span.con & ~mask) | ((layerIndex & 0x3f) << shift);
};
/**
 * Helper function to get connection data from a span
 */
const getCon = (span, dir) => {
    const shift = dir * 6; // 6 bits per direction
    return (span.con >> shift) & 0x3f;
};
/**
 * Count the number of walkable spans in the heightfield
 */
const getHeightFieldSpanCount = (heightfield) => {
    const numCols = heightfield.width * heightfield.height;
    let spanCount = 0;
    for (let columnIndex = 0; columnIndex < numCols; ++columnIndex) {
        let span = heightfield.spans[columnIndex];
        while (span != null) {
            if (span.area !== NULL_AREA) {
                spanCount++;
            }
            span = span.next || null;
        }
    }
    return spanCount;
};
const buildCompactHeightfield = (ctx, walkableHeightVoxels, walkableClimbVoxels, heightfield) => {
    const xSize = heightfield.width;
    const zSize = heightfield.height;
    const spanCount = getHeightFieldSpanCount(heightfield);
    const compactHeightfield = {
        width: xSize,
        height: zSize,
        spanCount,
        walkableHeightVoxels,
        walkableClimbVoxels,
        borderSize: 0,
        maxDistance: 0,
        maxRegions: 0,
        bounds: box3.clone(heightfield.bounds),
        cellSize: heightfield.cellSize,
        cellHeight: heightfield.cellHeight,
        cells: new Array(xSize * zSize),
        spans: new Array(spanCount),
        areas: new Array(spanCount),
        distances: new Array(spanCount).fill(0),
    };
    // adjust upper bound to account for walkable height
    compactHeightfield.bounds[1][1] += walkableHeightVoxels * heightfield.cellHeight;
    // initialize cells
    for (let i = 0; i < xSize * zSize; i++) {
        compactHeightfield.cells[i] = {
            index: 0,
            count: 0,
        };
    }
    // initialize spans
    for (let i = 0; i < spanCount; i++) {
        compactHeightfield.spans[i] = {
            y: 0,
            region: 0,
            con: 0,
            h: 0,
        };
        compactHeightfield.areas[i] = NULL_AREA;
    }
    // fill in cells and spans
    let currentCellIndex = 0;
    const numColumns = xSize * zSize;
    for (let columnIndex = 0; columnIndex < numColumns; ++columnIndex) {
        let span = heightfield.spans[columnIndex];
        // if there are no spans at this cell, just leave the data to index=0, count=0.
        if (span == null) {
            continue;
        }
        const cell = compactHeightfield.cells[columnIndex];
        cell.index = currentCellIndex;
        cell.count = 0;
        while (span != null) {
            if (span.area !== NULL_AREA) {
                const bot = span.max;
                const top = span.next ? span.next.min : MAX_HEIGHT;
                compactHeightfield.spans[currentCellIndex].y = Math.min(Math.max(bot, 0), 0xffff);
                compactHeightfield.spans[currentCellIndex].h = Math.min(Math.max(top - bot, 0), 0xff);
                compactHeightfield.areas[currentCellIndex] = span.area;
                currentCellIndex++;
                cell.count++;
            }
            span = span.next || null;
        }
    }
    // find neighbour connections
    let maxLayerIndex = 0;
    const zStride = xSize;
    for (let z = 0; z < zSize; ++z) {
        for (let x = 0; x < xSize; ++x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            for (let i = cell.index; i < cell.index + cell.count; ++i) {
                const span = compactHeightfield.spans[i];
                for (let dir = 0; dir < 4; ++dir) {
                    setCon(span, dir, NOT_CONNECTED);
                    const neighborX = x + DIR_OFFSETS[dir][0];
                    const neighborZ = z + DIR_OFFSETS[dir][1];
                    // first check that the neighbour cell is in bounds.
                    if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize) {
                        continue;
                    }
                    // iterate over all neighbour spans and check if any of them is
                    // accessible from current cell.
                    const neighborCell = compactHeightfield.cells[neighborX + neighborZ * zStride];
                    for (let k = neighborCell.index; k < neighborCell.index + neighborCell.count; ++k) {
                        const neighborSpan = compactHeightfield.spans[k];
                        const bot = Math.max(span.y, neighborSpan.y);
                        const top = Math.min(span.y + span.h, neighborSpan.y + neighborSpan.h);
                        // check that the gap between the spans is walkable,
                        // and that the climb height between the gaps is not too high.
                        if (top - bot >= walkableHeightVoxels && Math.abs(neighborSpan.y - span.y) <= walkableClimbVoxels) {
                            // Mark direction as walkable.
                            const layerIndex = k - neighborCell.index;
                            if (layerIndex < 0 || layerIndex > MAX_LAYERS) {
                                maxLayerIndex = Math.max(maxLayerIndex, layerIndex);
                                continue;
                            }
                            setCon(span, dir, layerIndex);
                            break;
                        }
                    }
                }
            }
        }
    }
    if (maxLayerIndex > MAX_LAYERS) {
        BuildContext.warn(ctx, `buildCompactHeightfield: Heightfield has too many layers ${maxLayerIndex} (max: ${MAX_LAYERS})`);
    }
    return compactHeightfield;
};
const MAX_DISTANCE = 255;
/**
 * Computes a distance field for the compact heightfield.
 * Each span gets a distance value representing how far it is from any boundary or obstacle.
 * @returns A Uint8Array containing distance values for each span
 */
const computeDistanceToBoundary = (compactHeightfield) => {
    const xSize = compactHeightfield.width;
    const zSize = compactHeightfield.height;
    const zStride = xSize; // for readability
    // initialize distance array
    const distanceToBoundary = new Uint8Array(compactHeightfield.spanCount);
    distanceToBoundary.fill(MAX_DISTANCE);
    // mark boundary cells
    for (let z = 0; z < zSize; ++z) {
        for (let x = 0; x < xSize; ++x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            for (let spanIndex = cell.index; spanIndex < cell.index + cell.count; ++spanIndex) {
                if (compactHeightfield.areas[spanIndex] === NULL_AREA) {
                    distanceToBoundary[spanIndex] = 0;
                    continue;
                }
                const span = compactHeightfield.spans[spanIndex];
                // check that there is a non-null adjacent span in each of the 4 cardinal directions
                let neighborCount = 0;
                for (let direction = 0; direction < 4; ++direction) {
                    const neighborConnection = getCon(span, direction);
                    if (neighborConnection === NOT_CONNECTED) {
                        break;
                    }
                    const neighborX = x + DIR_OFFSETS[direction][0];
                    const neighborZ = z + DIR_OFFSETS[direction][1];
                    const neighborSpanIndex = compactHeightfield.cells[neighborX + neighborZ * zStride].index + neighborConnection;
                    if (compactHeightfield.areas[neighborSpanIndex] === NULL_AREA) {
                        break;
                    }
                    neighborCount++;
                }
                // at least one missing neighbour, so this is a boundary cell
                if (neighborCount !== 4) {
                    distanceToBoundary[spanIndex] = 0;
                }
            }
        }
    }
    // pass 1: Forward pass (top-left to bottom-right)
    for (let z = 0; z < zSize; ++z) {
        for (let x = 0; x < xSize; ++x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            const maxSpanIndex = cell.index + cell.count;
            for (let spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
                const span = compactHeightfield.spans[spanIndex];
                if (getCon(span, 0) !== NOT_CONNECTED) {
                    // (-1,0) - West neighbor
                    const aX = x + DIR_OFFSETS[0][0];
                    const aY = z + DIR_OFFSETS[0][1];
                    const aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 0);
                    const aSpan = compactHeightfield.spans[aIndex];
                    let newDistance = Math.min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[spanIndex]) {
                        distanceToBoundary[spanIndex] = newDistance;
                    }
                    // (-1,-1) - Northwest diagonal
                    if (getCon(aSpan, 3) !== NOT_CONNECTED) {
                        const bX = aX + DIR_OFFSETS[3][0];
                        const bY = aY + DIR_OFFSETS[3][1];
                        const bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 3);
                        newDistance = Math.min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[spanIndex]) {
                            distanceToBoundary[spanIndex] = newDistance;
                        }
                    }
                }
                if (getCon(span, 3) !== NOT_CONNECTED) {
                    // (0,-1) - North neighbor
                    const aX = x + DIR_OFFSETS[3][0];
                    const aY = z + DIR_OFFSETS[3][1];
                    const aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 3);
                    const aSpan = compactHeightfield.spans[aIndex];
                    let newDistance = Math.min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[spanIndex]) {
                        distanceToBoundary[spanIndex] = newDistance;
                    }
                    // (1,-1) - Northeast diagonal
                    if (getCon(aSpan, 2) !== NOT_CONNECTED) {
                        const bX = aX + DIR_OFFSETS[2][0];
                        const bY = aY + DIR_OFFSETS[2][1];
                        const bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 2);
                        newDistance = Math.min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[spanIndex]) {
                            distanceToBoundary[spanIndex] = newDistance;
                        }
                    }
                }
            }
        }
    }
    // pass 2: Backward pass (bottom-right to top-left)
    for (let z = zSize - 1; z >= 0; --z) {
        for (let x = xSize - 1; x >= 0; --x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            const maxSpanIndex = cell.index + cell.count;
            for (let spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
                const span = compactHeightfield.spans[spanIndex];
                if (getCon(span, 2) !== NOT_CONNECTED) {
                    // (1,0) - East neighbor
                    const aX = x + DIR_OFFSETS[2][0];
                    const aY = z + DIR_OFFSETS[2][1];
                    const aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 2);
                    const aSpan = compactHeightfield.spans[aIndex];
                    let newDistance = Math.min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[spanIndex]) {
                        distanceToBoundary[spanIndex] = newDistance;
                    }
                    // (1,1) - Southeast diagonal
                    if (getCon(aSpan, 1) !== NOT_CONNECTED) {
                        const bX = aX + DIR_OFFSETS[1][0];
                        const bY = aY + DIR_OFFSETS[1][1];
                        const bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 1);
                        newDistance = Math.min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[spanIndex]) {
                            distanceToBoundary[spanIndex] = newDistance;
                        }
                    }
                }
                if (getCon(span, 1) !== NOT_CONNECTED) {
                    // (0,1) - South neighbor
                    const aX = x + DIR_OFFSETS[1][0];
                    const aY = z + DIR_OFFSETS[1][1];
                    const aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 1);
                    const aSpan = compactHeightfield.spans[aIndex];
                    let newDistance = Math.min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[spanIndex]) {
                        distanceToBoundary[spanIndex] = newDistance;
                    }
                    // (-1,1) - Southwest diagonal
                    if (getCon(aSpan, 0) !== NOT_CONNECTED) {
                        const bX = aX + DIR_OFFSETS[0][0];
                        const bY = aY + DIR_OFFSETS[0][1];
                        const bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 0);
                        newDistance = Math.min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[spanIndex]) {
                            distanceToBoundary[spanIndex] = newDistance;
                        }
                    }
                }
            }
        }
    }
    return distanceToBoundary;
};
const erodeWalkableArea = (walkableRadiusVoxels, compactHeightfield) => {
    const distanceToBoundary = computeDistanceToBoundary(compactHeightfield);
    // erode areas that are too close to boundaries
    const minBoundaryDistance = walkableRadiusVoxels * 2;
    for (let spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex) {
        if (distanceToBoundary[spanIndex] < minBoundaryDistance) {
            compactHeightfield.areas[spanIndex] = NULL_AREA;
        }
    }
};
/**
 * Erodes the walkable area for a base agent radius and marks restricted areas for larger agents based on given walkable radius thresholds.
 *
 * Note that this function requires careful tuning of the build parameters to get a good result:
 * - The cellSize needs to be small enough to accurately represent narrow passages. Generally you need to use smaller cellSizes than you otherwise would for single agent navmesh builds.
 * - The thresholds should not be so small that the resulting regions are too small to successfully build good navmesh polygons for. Values like 1-2 voxels will likely lead to poor results.
 * - You may get a better result using "buildRegionsMonotone" over "buildRegions" as this will better handle the many small clusters of areas that may be created from smaller thresholds.
 *
 * A typical workflow for using this utility to implement multi-agent support:
 * 1. Call erodeAndMarkWalkableAreas with your smallest agent radius and list of restricted areas
 * 2. Continue with buildDistanceField, buildRegionsMonotone, etc.
 * 3. Configure query filters so large agents exclude the narrow/restricted area IDs
 *
 * @param baseWalkableRadiusVoxels the smallest agent radius in voxels (used for erosion)
 * @param thresholds array of area ids and their corresponding walkable radius in voxels.
 * @param compactHeightfield the compact heightfield to process
 */
const erodeAndMarkWalkableAreas = (baseWalkableRadiusVoxels, thresholds, compactHeightfield) => {
    // compute distance field once for both operations
    const distanceToBoundary = computeDistanceToBoundary(compactHeightfield);
    // sort thresholds by radius (smallest first) - we want to mark narrowest corridors first
    const sortedThresholds = [...thresholds].sort((a, b) => a.walkableRadiusVoxels - b.walkableRadiusVoxels);
    const baseMinDistance = baseWalkableRadiusVoxels * 2;
    // process each span
    for (let spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex) {
        const distance = distanceToBoundary[spanIndex];
        // first, check if this span should be eroded (removed) based on base agent radius
        if (distance < baseMinDistance) {
            compactHeightfield.areas[spanIndex] = NULL_AREA;
            continue;
        }
        // span survived base erosion, now check if it should be marked
        for (const config of sortedThresholds) {
            const minDistance = config.walkableRadiusVoxels * 2;
            if (distance < minDistance) {
                // this span is too narrow for this agent size
                // mark it with the area id
                compactHeightfield.areas[spanIndex] = config.areaId;
                break; // once marked, we're done with this span
            }
        }
        // if the span wasn't eroded or marked, it remains in its current area
    }
};
/**
 * Marks spans in the heightfield that intersect the specified box area with the given area ID.
 */
const markBoxArea = (bounds, areaId, compactHeightfield) => {
    const [boxMinBounds, boxMaxBounds] = bounds;
    const xSize = compactHeightfield.width;
    const zSize = compactHeightfield.height;
    const zStride = xSize; // For readability
    // Find the footprint of the box area in grid cell coordinates.
    let minX = Math.floor((boxMinBounds[0] - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const minY = Math.floor((boxMinBounds[1] - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let minZ = Math.floor((boxMinBounds[2] - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    let maxX = Math.floor((boxMaxBounds[0] - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const maxY = Math.floor((boxMaxBounds[1] - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let maxZ = Math.floor((boxMaxBounds[2] - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    // Early-out if the box is outside the bounds of the grid.
    if (maxX < 0)
        return;
    if (minX >= xSize)
        return;
    if (maxZ < 0)
        return;
    if (minZ >= zSize)
        return;
    // Clamp relevant bound coordinates to the grid.
    if (minX < 0)
        minX = 0;
    if (maxX >= xSize)
        maxX = xSize - 1;
    if (minZ < 0)
        minZ = 0;
    if (maxZ >= zSize)
        maxZ = zSize - 1;
    // Mark relevant cells.
    for (let z = minZ; z <= maxZ; ++z) {
        for (let x = minX; x <= maxX; ++x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            const maxSpanIndex = cell.index + cell.count;
            for (let spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
                const span = compactHeightfield.spans[spanIndex];
                // Skip if the span is outside the box extents.
                if (span.y < minY || span.y > maxY) {
                    continue;
                }
                // Skip if the span has been removed.
                if (compactHeightfield.areas[spanIndex] === NULL_AREA) {
                    continue;
                }
                // Mark the span.
                compactHeightfield.areas[spanIndex] = areaId;
            }
        }
    }
};
/**
 * Marks spans in the heightfield that intersect the specified rotated box area with the given area ID.
 * @param center - The center point of the box in world space [x, y, z]
 * @param halfExtents - Half extents of the box along each axis [x, y, z]
 * @param angleRadians - Rotation angle in radians around the Y axis
 * @param areaId - The area ID to assign to intersecting spans
 * @param compactHeightfield - The compact heightfield to mark
 */
const markRotatedBoxArea = (center, halfExtents, angleRadians, areaId, compactHeightfield) => {
    const xSize = compactHeightfield.width;
    const zSize = compactHeightfield.height;
    const zStride = xSize; // for readability
    // precompute sin and cos for rotation
    const cosAngle = Math.cos(angleRadians);
    const sinAngle = Math.sin(angleRadians);
    // compute the 4 corners of the rotated box in the XZ plane and find AABB
    // the corners in local space are at (±halfExtents[0], ±halfExtents[2])
    const hx = halfExtents[0];
    const hz = halfExtents[2];
    // corner 1: (-hx, -hz)
    let worldX = center[0] + cosAngle * -hx - sinAngle * -hz;
    let worldZ = center[2] + sinAngle * -hx + cosAngle * -hz;
    let minWorldX = worldX;
    let maxWorldX = worldX;
    let minWorldZ = worldZ;
    let maxWorldZ = worldZ;
    // corner 2: (hx, -hz)
    worldX = center[0] + cosAngle * hx - sinAngle * -hz;
    worldZ = center[2] + sinAngle * hx + cosAngle * -hz;
    minWorldX = Math.min(minWorldX, worldX);
    maxWorldX = Math.max(maxWorldX, worldX);
    minWorldZ = Math.min(minWorldZ, worldZ);
    maxWorldZ = Math.max(maxWorldZ, worldZ);
    // corner 3: (hx, hz)
    worldX = center[0] + cosAngle * hx - sinAngle * hz;
    worldZ = center[2] + sinAngle * hx + cosAngle * hz;
    minWorldX = Math.min(minWorldX, worldX);
    maxWorldX = Math.max(maxWorldX, worldX);
    minWorldZ = Math.min(minWorldZ, worldZ);
    maxWorldZ = Math.max(maxWorldZ, worldZ);
    // corner 4: (-hx, hz)
    worldX = center[0] + cosAngle * -hx - sinAngle * hz;
    worldZ = center[2] + sinAngle * -hx + cosAngle * hz;
    minWorldX = Math.min(minWorldX, worldX);
    maxWorldX = Math.max(maxWorldX, worldX);
    minWorldZ = Math.min(minWorldZ, worldZ);
    maxWorldZ = Math.max(maxWorldZ, worldZ);
    // compute Y extents in world space
    const minWorldY = center[1] - halfExtents[1];
    const maxWorldY = center[1] + halfExtents[1];
    // convert AABB to grid coordinates
    let minX = Math.floor((minWorldX - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const minY = Math.floor((minWorldY - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let minZ = Math.floor((minWorldZ - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    let maxX = Math.floor((maxWorldX - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const maxY = Math.floor((maxWorldY - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let maxZ = Math.floor((maxWorldZ - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    // early-out if the rotated box AABB is outside the grid bounds
    if (maxX < 0)
        return;
    if (minX >= xSize)
        return;
    if (maxZ < 0)
        return;
    if (minZ >= zSize)
        return;
    // clamp to grid bounds
    if (minX < 0)
        minX = 0;
    if (maxX >= xSize)
        maxX = xSize - 1;
    if (minZ < 0)
        minZ = 0;
    if (maxZ >= zSize)
        maxZ = zSize - 1;
    // iterate through cells in the AABB
    for (let z = minZ; z <= maxZ; ++z) {
        for (let x = minX; x <= maxX; ++x) {
            // calculate cell center in world space
            const cellWorldX = compactHeightfield.bounds[0][0] + (x + 0.5) * compactHeightfield.cellSize;
            const cellWorldZ = compactHeightfield.bounds[0][2] + (z + 0.5) * compactHeightfield.cellSize;
            // transform cell center to box's local coordinate system
            // first translate to box origin
            const dx = cellWorldX - center[0];
            const dz = cellWorldZ - center[2];
            // then apply inverse rotation (rotation by -angleRadians)
            // inverse rotation matrix for Y-axis: [cos(θ), -sin(θ); sin(θ), cos(θ)]
            const localX = cosAngle * dx - sinAngle * dz;
            const localZ = sinAngle * dx + cosAngle * dz;
            // check if the point is inside the box in local space
            if (Math.abs(localX) > halfExtents[0] || Math.abs(localZ) > halfExtents[2]) {
                continue;
            }
            // cell is inside the rotated box, mark its spans
            const cell = compactHeightfield.cells[x + z * zStride];
            const maxSpanIndex = cell.index + cell.count;
            for (let spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
                const span = compactHeightfield.spans[spanIndex];
                // skip if the span is outside the Y extents
                if (span.y < minY || span.y > maxY) {
                    continue;
                }
                // skip if the span has been removed
                if (compactHeightfield.areas[spanIndex] === NULL_AREA) {
                    continue;
                }
                // mark the span
                compactHeightfield.areas[spanIndex] = areaId;
            }
        }
    }
};
const _markConvexPolyArea_point = vec3.create();
/**
 * Marks spans in the heightfield that intersect the specified convex polygon area with the given area ID.
 */
const markConvexPolyArea = (verts, minY, maxY, areaId, compactHeightfield) => {
    const xSize = compactHeightfield.width;
    const zSize = compactHeightfield.height;
    const zStride = xSize; // for readability
    // compute the bounding box of the polygon
    const bmin = [verts[0], minY, verts[2]];
    const bmax = [verts[0], maxY, verts[2]];
    const numVerts = verts.length / 3;
    for (let i = 1; i < numVerts; ++i) {
        const vertIndex = i * 3;
        bmin[0] = Math.min(bmin[0], verts[vertIndex]);
        bmin[2] = Math.min(bmin[2], verts[vertIndex + 2]);
        bmax[0] = Math.max(bmax[0], verts[vertIndex]);
        bmax[2] = Math.max(bmax[2], verts[vertIndex + 2]);
    }
    // compute the grid footprint of the polygon
    let minx = Math.floor((bmin[0] - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const miny = Math.floor((bmin[1] - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let minz = Math.floor((bmin[2] - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    let maxx = Math.floor((bmax[0] - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const maxy = Math.floor((bmax[1] - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let maxz = Math.floor((bmax[2] - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    // early-out if the polygon lies entirely outside the grid.
    if (maxx < 0)
        return;
    if (minx >= xSize)
        return;
    if (maxz < 0)
        return;
    if (minz >= zSize)
        return;
    // clamp the polygon footprint to the grid
    if (minx < 0)
        minx = 0;
    if (maxx >= xSize)
        maxx = xSize - 1;
    if (minz < 0)
        minz = 0;
    if (maxz >= zSize)
        maxz = zSize - 1;
    // TODO: optimize.
    for (let z = minz; z <= maxz; ++z) {
        for (let x = minx; x <= maxx; ++x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            const maxSpanIndex = cell.index + cell.count;
            for (let spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
                const span = compactHeightfield.spans[spanIndex];
                // skip if span is removed.
                if (compactHeightfield.areas[spanIndex] === NULL_AREA) {
                    continue;
                }
                // skip if y extents don't overlap.
                if (span.y < miny || span.y > maxy) {
                    continue;
                }
                const point = vec3.set(_markConvexPolyArea_point, compactHeightfield.bounds[0][0] + (x + 0.5) * compactHeightfield.cellSize, 0, compactHeightfield.bounds[0][2] + (z + 0.5) * compactHeightfield.cellSize);
                if (pointInPoly(point, verts, numVerts)) {
                    compactHeightfield.areas[spanIndex] = areaId;
                }
            }
        }
    }
};
/**
 * Marks spans in the heightfield that intersect the specified cylinder area with the given area ID.
 */
const markCylinderArea = (position, radius, height, areaId, compactHeightfield) => {
    const xSize = compactHeightfield.width;
    const zSize = compactHeightfield.height;
    const zStride = xSize; // for readability
    // compute the bounding box of the cylinder
    const cylinderBBMin = [position[0] - radius, position[1], position[2] - radius];
    const cylinderBBMax = [position[0] + radius, position[1] + height, position[2] + radius];
    // compute the grid footprint of the cylinder
    let minx = Math.floor((cylinderBBMin[0] - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const miny = Math.floor((cylinderBBMin[1] - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let minz = Math.floor((cylinderBBMin[2] - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    let maxx = Math.floor((cylinderBBMax[0] - compactHeightfield.bounds[0][0]) / compactHeightfield.cellSize);
    const maxy = Math.floor((cylinderBBMax[1] - compactHeightfield.bounds[0][1]) / compactHeightfield.cellHeight);
    let maxz = Math.floor((cylinderBBMax[2] - compactHeightfield.bounds[0][2]) / compactHeightfield.cellSize);
    // early-out if the cylinder is completely outside the grid bounds.
    if (maxx < 0 || minx >= xSize || maxz < 0 || minz >= zSize) {
        return;
    }
    // clamp the cylinder bounds to the grid.
    if (minx < 0)
        minx = 0;
    if (maxx >= xSize)
        maxx = xSize - 1;
    if (minz < 0)
        minz = 0;
    if (maxz >= zSize)
        maxz = zSize - 1;
    const radiusSq = radius * radius;
    for (let z = minz; z <= maxz; ++z) {
        for (let x = minx; x <= maxx; ++x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            const maxSpanIndex = cell.index + cell.count;
            const cellX = compactHeightfield.bounds[0][0] + (x + 0.5) * compactHeightfield.cellSize;
            const cellZ = compactHeightfield.bounds[0][2] + (z + 0.5) * compactHeightfield.cellSize;
            const deltaX = cellX - position[0];
            const deltaZ = cellZ - position[2];
            // skip this column if it's too far from the center point of the cylinder.
            if (deltaX * deltaX + deltaZ * deltaZ >= radiusSq) {
                continue;
            }
            // mark all overlapping spans
            for (let spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
                const span = compactHeightfield.spans[spanIndex];
                // skip if span is removed.
                if (compactHeightfield.areas[spanIndex] === NULL_AREA) {
                    continue;
                }
                // mark if y extents overlap.
                if (span.y >= miny && span.y <= maxy) {
                    compactHeightfield.areas[spanIndex] = areaId;
                }
            }
        }
    }
};
/**
 * Helper function to perform insertion sort on a small array
 */
const insertSort = (arr, length) => {
    for (let i = 1; i < length; ++i) {
        const key = arr[i];
        let j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
};
const _neighborAreas = new Array(9);
/**
 * Applies a median filter to walkable area types (based on area id), removing noise.
 * filter is usually applied after applying area id's using functions
 * such as #markBoxArea, #markConvexPolyArea, and #markCylinderArea.
 */
const medianFilterWalkableArea = (compactHeightfield) => {
    const xSize = compactHeightfield.width;
    const zSize = compactHeightfield.height;
    const zStride = xSize; // for readability
    // create a temporary array to store the filtered areas
    const areas = new Uint8Array(compactHeightfield.spanCount);
    areas.fill(0xff);
    for (let z = 0; z < zSize; ++z) {
        for (let x = 0; x < xSize; ++x) {
            const cell = compactHeightfield.cells[x + z * zStride];
            const maxSpanIndex = cell.index + cell.count;
            for (let spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
                const span = compactHeightfield.spans[spanIndex];
                if (compactHeightfield.areas[spanIndex] === NULL_AREA) {
                    areas[spanIndex] = compactHeightfield.areas[spanIndex];
                    continue;
                }
                // collect neighbor areas (including center cell)
                for (let neighborIndex = 0; neighborIndex < 9; ++neighborIndex) {
                    _neighborAreas[neighborIndex] = compactHeightfield.areas[spanIndex];
                }
                // check all 4 cardinal directions
                for (let dir = 0; dir < 4; ++dir) {
                    if (getCon(span, dir) === NOT_CONNECTED) {
                        continue;
                    }
                    const aX = x + DIR_OFFSETS[dir][0];
                    const aZ = z + DIR_OFFSETS[dir][1];
                    const aIndex = compactHeightfield.cells[aX + aZ * zStride].index + getCon(span, dir);
                    if (compactHeightfield.areas[aIndex] !== NULL_AREA) {
                        _neighborAreas[dir * 2 + 0] = compactHeightfield.areas[aIndex];
                    }
                    // check diagonal neighbor
                    const aSpan = compactHeightfield.spans[aIndex];
                    const dir2 = (dir + 1) & 0x3;
                    const neighborConnection2 = getCon(aSpan, dir2);
                    if (neighborConnection2 !== NOT_CONNECTED) {
                        const bX = aX + DIR_OFFSETS[dir2][0];
                        const bZ = aZ + DIR_OFFSETS[dir2][1];
                        const bIndex = compactHeightfield.cells[bX + bZ * zStride].index + neighborConnection2;
                        if (compactHeightfield.areas[bIndex] !== NULL_AREA) {
                            _neighborAreas[dir * 2 + 1] = compactHeightfield.areas[bIndex];
                        }
                    }
                }
                // sort and take median (middle value)
                insertSort(_neighborAreas, 9);
                areas[spanIndex] = _neighborAreas[4];
            }
        }
    }
    // Copy filtered areas back to the heightfield
    for (let i = 0; i < compactHeightfield.spanCount; ++i) {
        compactHeightfield.areas[i] = areas[i];
    }
    return true;
};

const LOG_NB_STACKS = 3;
const NB_STACKS = 1 << LOG_NB_STACKS;
const EXPAND_ITERS = 8;
/**
 * Calculate distance field using a two-pass distance transform algorithm
 */
const calculateDistanceField = (compactHeightfield, distances) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    // initialize distance values to maximum
    distances.fill(0xffff);
    // mark boundary cells
    for (let y = 0; y < h; ++y) {
        for (let x = 0; x < w; ++x) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; ++i) {
                const span = compactHeightfield.spans[i];
                const area = compactHeightfield.areas[i];
                let neighborCount = 0;
                for (let dir = 0; dir < 4; ++dir) {
                    if (getCon(span, dir) !== NOT_CONNECTED) {
                        const ax = x + DIR_OFFSETS[dir][0];
                        const ay = y + DIR_OFFSETS[dir][1];
                        const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
                        if (area === compactHeightfield.areas[ai]) {
                            neighborCount++;
                        }
                    }
                }
                if (neighborCount !== 4) {
                    distances[i] = 0;
                }
            }
        }
    }
    // pass 1: forward pass
    for (let y = 0; y < h; ++y) {
        for (let x = 0; x < w; ++x) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; ++i) {
                const span = compactHeightfield.spans[i];
                if (getCon(span, 0) !== NOT_CONNECTED) {
                    // (-1,0) - west
                    const ax = x + DIR_OFFSETS[0][0];
                    const ay = y + DIR_OFFSETS[0][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 0);
                    const aSpan = compactHeightfield.spans[ai];
                    if (distances[ai] + 2 < distances[i]) {
                        distances[i] = distances[ai] + 2;
                    }
                    // (-1,-1) - northwest
                    if (getCon(aSpan, 3) !== NOT_CONNECTED) {
                        const aax = ax + DIR_OFFSETS[3][0];
                        const aay = ay + DIR_OFFSETS[3][1];
                        const aai = compactHeightfield.cells[aax + aay * w].index + getCon(aSpan, 3);
                        if (distances[aai] + 3 < distances[i]) {
                            distances[i] = distances[aai] + 3;
                        }
                    }
                }
                if (getCon(span, 3) !== NOT_CONNECTED) {
                    // (0,-1) - north
                    const ax = x + DIR_OFFSETS[3][0];
                    const ay = y + DIR_OFFSETS[3][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 3);
                    const aSpan = compactHeightfield.spans[ai];
                    if (distances[ai] + 2 < distances[i]) {
                        distances[i] = distances[ai] + 2;
                    }
                    // (1,-1) - northeast
                    if (getCon(aSpan, 2) !== NOT_CONNECTED) {
                        const aax = ax + DIR_OFFSETS[2][0];
                        const aay = ay + DIR_OFFSETS[2][1];
                        const aai = compactHeightfield.cells[aax + aay * w].index + getCon(aSpan, 2);
                        if (distances[aai] + 3 < distances[i]) {
                            distances[i] = distances[aai] + 3;
                        }
                    }
                }
            }
        }
    }
    // pass 2: backward pass
    for (let y = h - 1; y >= 0; --y) {
        for (let x = w - 1; x >= 0; --x) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; ++i) {
                const span = compactHeightfield.spans[i];
                if (getCon(span, 2) !== NOT_CONNECTED) {
                    // (1,0) - east
                    const ax = x + DIR_OFFSETS[2][0];
                    const ay = y + DIR_OFFSETS[2][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 2);
                    const aSpan = compactHeightfield.spans[ai];
                    if (distances[ai] + 2 < distances[i]) {
                        distances[i] = distances[ai] + 2;
                    }
                    // (1,1) - southeast
                    if (getCon(aSpan, 1) !== NOT_CONNECTED) {
                        const aax = ax + DIR_OFFSETS[1][0];
                        const aay = ay + DIR_OFFSETS[1][1];
                        const aai = compactHeightfield.cells[aax + aay * w].index + getCon(aSpan, 1);
                        if (distances[aai] + 3 < distances[i]) {
                            distances[i] = distances[aai] + 3;
                        }
                    }
                }
                if (getCon(span, 1) !== NOT_CONNECTED) {
                    // (0,1) - south
                    const ax = x + DIR_OFFSETS[1][0];
                    const ay = y + DIR_OFFSETS[1][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 1);
                    const aSpan = compactHeightfield.spans[ai];
                    if (distances[ai] + 2 < distances[i]) {
                        distances[i] = distances[ai] + 2;
                    }
                    // (-1,1) - southwest
                    if (getCon(aSpan, 0) !== NOT_CONNECTED) {
                        const aax = ax + DIR_OFFSETS[0][0];
                        const aay = ay + DIR_OFFSETS[0][1];
                        const aai = compactHeightfield.cells[aax + aay * w].index + getCon(aSpan, 0);
                        if (distances[aai] + 3 < distances[i]) {
                            distances[i] = distances[aai] + 3;
                        }
                    }
                }
            }
        }
    }
    // find maximum distance
    let maxDist = 0;
    for (let i = 0; i < compactHeightfield.spanCount; ++i) {
        maxDist = Math.max(distances[i], maxDist);
    }
    return maxDist;
};
/**
 * Apply box blur filter to smooth distance values
 */
const boxBlur = (compactHeightfield, threshold, srcDistances, dstDistances) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    const scaledThreshold = threshold * 2;
    for (let y = 0; y < h; ++y) {
        for (let x = 0; x < w; ++x) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; ++i) {
                const span = compactHeightfield.spans[i];
                const cd = srcDistances[i];
                if (cd <= scaledThreshold) {
                    dstDistances[i] = cd;
                    continue;
                }
                let d = cd;
                for (let dir = 0; dir < 4; ++dir) {
                    if (getCon(span, dir) !== NOT_CONNECTED) {
                        const ax = x + DIR_OFFSETS[dir][0];
                        const ay = y + DIR_OFFSETS[dir][1];
                        const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
                        d += srcDistances[ai];
                        const aSpan = compactHeightfield.spans[ai];
                        const dir2 = (dir + 1) & 0x3;
                        if (getCon(aSpan, dir2) !== NOT_CONNECTED) {
                            const ax2 = ax + DIR_OFFSETS[dir2][0];
                            const ay2 = ay + DIR_OFFSETS[dir2][1];
                            const ai2 = compactHeightfield.cells[ax2 + ay2 * w].index + getCon(aSpan, dir2);
                            d += srcDistances[ai2];
                        }
                        else {
                            d += cd;
                        }
                    }
                    else {
                        d += cd * 2;
                    }
                }
                dstDistances[i] = Math.floor((d + 5) / 9);
            }
        }
    }
};
const buildDistanceField = (compactHeightfield) => {
    // create temporary array for blurring
    const tempDistances = new Array(compactHeightfield.spanCount).fill(0);
    // calculate distance field directly into the heightfield's distances array
    const maxDist = calculateDistanceField(compactHeightfield, compactHeightfield.distances);
    compactHeightfield.maxDistance = maxDist;
    // apply box blur
    boxBlur(compactHeightfield, 1, compactHeightfield.distances, tempDistances);
    // copy the box blur result back to the heightfield
    for (let i = 0; i < compactHeightfield.spanCount; i++) {
        compactHeightfield.distances[i] = tempDistances[i];
    }
};
const buildRegions = (ctx, compactHeightfield, borderSize, minRegionArea, mergeRegionArea) => {
    // region building constants
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    // initialize region and distance buffers
    const srcReg = new Array(compactHeightfield.spanCount).fill(0);
    const srcDist = new Array(compactHeightfield.spanCount).fill(0);
    let regionId = 1;
    let level = (compactHeightfield.maxDistance + 1) & -2;
    // initialize level stacks
    const lvlStacks = [];
    for (let i = 0; i < NB_STACKS; i++) {
        lvlStacks[i] = [];
    }
    const stack = [];
    // paint border regions if border size is specified
    if (borderSize > 0) {
        const bw = Math.min(w, borderSize);
        const bh = Math.min(h, borderSize);
        // paint border rectangles
        paintRectRegion(0, bw, 0, h, regionId | BORDER_REG, compactHeightfield, srcReg);
        regionId++;
        paintRectRegion(w - bw, w, 0, h, regionId | BORDER_REG, compactHeightfield, srcReg);
        regionId++;
        paintRectRegion(0, w, 0, bh, regionId | BORDER_REG, compactHeightfield, srcReg);
        regionId++;
        paintRectRegion(0, w, h - bh, h, regionId | BORDER_REG, compactHeightfield, srcReg);
        regionId++;
    }
    compactHeightfield.borderSize = borderSize;
    let sId = -1;
    while (level > 0) {
        level = level >= 2 ? level - 2 : 0;
        sId = (sId + 1) & (NB_STACKS - 1);
        if (sId === 0) {
            sortCellsByLevel(level, compactHeightfield, srcReg, NB_STACKS, lvlStacks, 1);
        }
        else {
            appendStacks(lvlStacks[sId - 1], lvlStacks[sId], srcReg);
        }
        // expand current regions until no empty connected cells found
        expandRegions(EXPAND_ITERS, level, compactHeightfield, srcReg, srcDist, lvlStacks[sId], false);
        // mark new regions with IDs
        for (let j = 0; j < lvlStacks[sId].length; j++) {
            const current = lvlStacks[sId][j];
            const x = current.x;
            const y = current.y;
            const i = current.index;
            if (i >= 0 && srcReg[i] === 0) {
                if (floodRegion(x, y, i, level, regionId, compactHeightfield, srcReg, srcDist, stack)) {
                    if (regionId === 0xffff) {
                        BuildContext.error(ctx, 'Region ID overflow');
                        return false;
                    }
                    regionId++;
                }
            }
        }
    }
    // expand current regions until no empty connected cells found
    expandRegions(EXPAND_ITERS * 8, 0, compactHeightfield, srcReg, srcDist, stack, true);
    // merge regions and filter out small regions
    const overlaps = [];
    compactHeightfield.maxRegions = regionId;
    if (!mergeAndFilterRegions(minRegionArea, mergeRegionArea, compactHeightfield, srcReg, overlaps)) ;
    // write the result to compact heightfield spans
    for (let i = 0; i < compactHeightfield.spanCount; i++) {
        compactHeightfield.spans[i].region = srcReg[i];
    }
    return true;
};
/**
 * Paint a rectangular region with the given region ID
 */
const paintRectRegion = (minx, maxx, miny, maxy, regId, compactHeightfield, srcReg) => {
    const w = compactHeightfield.width;
    for (let y = miny; y < maxy; y++) {
        for (let x = minx; x < maxx; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                if (compactHeightfield.areas[i] !== NULL_AREA) {
                    srcReg[i] = regId;
                }
            }
        }
    }
};
/**
 * Sort cells by their distance level into stacks
 */
const sortCellsByLevel = (startLevel, compactHeightfield, srcReg, nbStacks, stacks, logLevelsPerStack) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    const adjustedStartLevel = startLevel >> logLevelsPerStack;
    // clear all stacks
    for (let j = 0; j < nbStacks; j++) {
        stacks[j].length = 0;
    }
    // put all cells in the level range into appropriate stacks
    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                if (compactHeightfield.areas[i] === NULL_AREA || srcReg[i] !== 0) {
                    continue;
                }
                const level = compactHeightfield.distances[i] >> logLevelsPerStack;
                let sId = adjustedStartLevel - level;
                if (sId >= nbStacks) {
                    continue;
                }
                if (sId < 0) {
                    sId = 0;
                }
                stacks[sId].push({ x, y, index: i });
            }
        }
    }
};
/**
 * Append entries from source stack to destination stack
 */
const appendStacks = (srcStack, dstStack, srcReg) => {
    for (let j = 0; j < srcStack.length; j++) {
        const entry = srcStack[j];
        if (entry.index < 0 || srcReg[entry.index] !== 0) {
            continue;
        }
        dstStack.push(entry);
    }
};
/**
 * Flood fill a region starting from a given point
 */
const floodRegion = (x, y, i, level, r, compactHeightfield, srcReg, srcDist, stack) => {
    const w = compactHeightfield.width;
    const area = compactHeightfield.areas[i];
    // flood fill mark region
    stack.length = 0;
    stack.push({ x, y, index: i });
    srcReg[i] = r;
    srcDist[i] = 0;
    const lev = level >= 2 ? level - 2 : 0;
    let count = 0;
    while (stack.length > 0) {
        const current = stack.pop();
        const cx = current.x;
        const cy = current.y;
        const ci = current.index;
        const span = compactHeightfield.spans[ci];
        // check if any neighbors already have a valid region set
        let ar = 0;
        for (let dir = 0; dir < 4; dir++) {
            if (getCon(span, dir) !== NOT_CONNECTED) {
                const ax = cx + DIR_OFFSETS[dir][0];
                const ay = cy + DIR_OFFSETS[dir][1];
                const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
                if (compactHeightfield.areas[ai] !== area) {
                    continue;
                }
                const nr = srcReg[ai];
                if (nr & BORDER_REG) {
                    continue;
                }
                if (nr !== 0 && nr !== r) {
                    ar = nr;
                    break;
                }
                const aSpan = compactHeightfield.spans[ai];
                const dir2 = (dir + 1) & 0x3;
                if (getCon(aSpan, dir2) !== NOT_CONNECTED) {
                    const ax2 = ax + DIR_OFFSETS[dir2][0];
                    const ay2 = ay + DIR_OFFSETS[dir2][1];
                    const ai2 = compactHeightfield.cells[ax2 + ay2 * w].index + getCon(aSpan, dir2);
                    if (compactHeightfield.areas[ai2] !== area) {
                        continue;
                    }
                    const nr2 = srcReg[ai2];
                    if (nr2 !== 0 && nr2 !== r) {
                        ar = nr2;
                        break;
                    }
                }
            }
        }
        if (ar !== 0) {
            srcReg[ci] = 0;
            continue;
        }
        count++;
        // expand neighbors
        for (let dir = 0; dir < 4; dir++) {
            if (getCon(span, dir) !== NOT_CONNECTED) {
                const ax = cx + DIR_OFFSETS[dir][0];
                const ay = cy + DIR_OFFSETS[dir][1];
                const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
                if (compactHeightfield.areas[ai] !== area) {
                    continue;
                }
                if (compactHeightfield.distances[ai] >= lev && srcReg[ai] === 0) {
                    srcReg[ai] = r;
                    srcDist[ai] = 0;
                    stack.push({ x: ax, y: ay, index: ai });
                }
            }
        }
    }
    return count > 0;
};
/**
 * Expand regions iteratively
 */
const expandRegions = (maxIter, level, compactHeightfield, srcReg, srcDist, stack, fillStack) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    if (fillStack) {
        // find cells revealed by the raised level
        stack.length = 0;
        for (let y = 0; y < h; y++) {
            for (let x = 0; x < w; x++) {
                const cell = compactHeightfield.cells[x + y * w];
                for (let i = cell.index; i < cell.index + cell.count; i++) {
                    if (compactHeightfield.distances[i] >= level &&
                        srcReg[i] === 0 &&
                        compactHeightfield.areas[i] !== NULL_AREA) {
                        stack.push({ x, y, index: i });
                    }
                }
            }
        }
    }
    else {
        // mark cells which already have a region
        for (let j = 0; j < stack.length; j++) {
            const i = stack[j].index;
            if (srcReg[i] !== 0) {
                stack[j].index = -1;
            }
        }
    }
    const dirtyEntries = [];
    let iter = 0;
    while (stack.length > 0) {
        let failed = 0;
        dirtyEntries.length = 0;
        for (let j = 0; j < stack.length; j++) {
            const x = stack[j].x;
            const y = stack[j].y;
            const i = stack[j].index;
            if (i < 0) {
                failed++;
                continue;
            }
            let r = srcReg[i];
            let d2 = 0xffff;
            const area = compactHeightfield.areas[i];
            const span = compactHeightfield.spans[i];
            for (let dir = 0; dir < 4; dir++) {
                if (getCon(span, dir) === NOT_CONNECTED)
                    continue;
                const ax = x + DIR_OFFSETS[dir][0];
                const ay = y + DIR_OFFSETS[dir][1];
                const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
                if (compactHeightfield.areas[ai] !== area)
                    continue;
                if (srcReg[ai] > 0 && (srcReg[ai] & BORDER_REG) === 0) {
                    if (srcDist[ai] + 2 < d2) {
                        r = srcReg[ai];
                        d2 = srcDist[ai] + 2;
                    }
                }
            }
            if (r) {
                stack[j].index = -1; // mark as used
                dirtyEntries.push({ index: i, region: r, distance2: d2 });
            }
            else {
                failed++;
            }
        }
        // copy entries that differ to keep them in sync
        for (let i = 0; i < dirtyEntries.length; i++) {
            const entry = dirtyEntries[i];
            srcReg[entry.index] = entry.region;
            srcDist[entry.index] = entry.distance2;
        }
        if (failed === stack.length) {
            break;
        }
        if (level > 0) {
            iter++;
            if (iter >= maxIter) {
                break;
            }
        }
    }
};
/**
 * Merge and filter regions based on size criteria
 */
const mergeAndFilterRegions = (minRegionArea, mergeRegionSize, compactHeightfield, srcReg, overlaps) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    const nreg = compactHeightfield.maxRegions + 1;
    // construct regions
    const regions = [];
    for (let i = 0; i < nreg; i++) {
        regions.push({
            spanCount: 0,
            id: i,
            areaType: 0,
            remap: false,
            visited: false,
            overlap: false,
            connectsToBorder: false,
            ymin: 0xffff,
            ymax: 0,
            connections: [],
            floors: [],
        });
    }
    // find edge of a region and find connections around the contour
    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                const r = srcReg[i];
                if (r === 0 || r >= nreg)
                    continue;
                const reg = regions[r];
                reg.spanCount++;
                // update floors
                for (let j = cell.index; j < cell.index + cell.count; j++) {
                    if (i === j)
                        continue;
                    const floorId = srcReg[j];
                    if (floorId === 0 || floorId >= nreg)
                        continue;
                    if (floorId === r) {
                        reg.overlap = true;
                    }
                    addUniqueFloorRegion(reg, floorId);
                }
                // have found contour
                if (reg.connections.length > 0)
                    continue;
                reg.areaType = compactHeightfield.areas[i];
                // check if this cell is next to a border
                let ndir = -1;
                for (let dir = 0; dir < 4; dir++) {
                    if (isSolidEdge(compactHeightfield, srcReg, x, y, i, dir)) {
                        ndir = dir;
                        break;
                    }
                }
                if (ndir !== -1) {
                    // the cell is at border - walk around the contour to find all neighbors
                    walkContour$1(x, y, i, ndir, compactHeightfield, srcReg, reg.connections);
                }
            }
        }
    }
    // remove too small regions
    const stack = [];
    const trace = [];
    for (let i = 0; i < nreg; i++) {
        const reg = regions[i];
        if (reg.id === 0 || reg.id & BORDER_REG)
            continue;
        if (reg.spanCount === 0)
            continue;
        if (reg.visited)
            continue;
        // count the total size of all connected regions
        let connectsToBorder = false;
        let spanCount = 0;
        stack.length = 0;
        trace.length = 0;
        reg.visited = true;
        stack.push(i);
        while (stack.length > 0) {
            const ri = stack.pop();
            const creg = regions[ri];
            spanCount += creg.spanCount;
            trace.push(ri);
            for (let j = 0; j < creg.connections.length; j++) {
                if (creg.connections[j] & BORDER_REG) {
                    connectsToBorder = true;
                    continue;
                }
                const neireg = regions[creg.connections[j]];
                if (neireg.visited)
                    continue;
                if (neireg.id === 0 || neireg.id & BORDER_REG)
                    continue;
                stack.push(neireg.id);
                neireg.visited = true;
            }
        }
        // if the accumulated region size is too small, remove it
        if (spanCount < minRegionArea && !connectsToBorder) {
            for (let j = 0; j < trace.length; j++) {
                regions[trace[j]].spanCount = 0;
                regions[trace[j]].id = 0;
            }
        }
    }
    // merge too small regions to neighbor regions
    let mergeCount = 0;
    do {
        mergeCount = 0;
        for (let i = 0; i < nreg; i++) {
            const reg = regions[i];
            if (reg.id === 0 || reg.id & BORDER_REG)
                continue;
            if (reg.overlap)
                continue;
            if (reg.spanCount === 0)
                continue;
            // check to see if the region should be merged
            if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg)) {
                continue;
            }
            // find smallest neighbor region that connects to this one
            let smallest = 0xfffffff;
            let mergeId = reg.id;
            for (let j = 0; j < reg.connections.length; j++) {
                if (reg.connections[j] & BORDER_REG)
                    continue;
                const mreg = regions[reg.connections[j]];
                if (mreg.id === 0 || mreg.id & BORDER_REG || mreg.overlap)
                    continue;
                if (mreg.spanCount < smallest && canMergeWithRegion(reg, mreg) && canMergeWithRegion(mreg, reg)) {
                    smallest = mreg.spanCount;
                    mergeId = mreg.id;
                }
            }
            // found new id
            if (mergeId !== reg.id) {
                const oldId = reg.id;
                const target = regions[mergeId];
                // merge neighbors
                if (mergeRegions(target, reg)) {
                    // fixup regions pointing to current region
                    for (let j = 0; j < nreg; j++) {
                        if (regions[j].id === 0 || regions[j].id & BORDER_REG)
                            continue;
                        if (regions[j].id === oldId) {
                            regions[j].id = mergeId;
                        }
                        replaceNeighbor(regions[j], oldId, mergeId);
                    }
                    mergeCount++;
                }
            }
        }
    } while (mergeCount > 0);
    // compress region IDs
    for (let i = 0; i < nreg; i++) {
        regions[i].remap = false;
        if (regions[i].id === 0)
            continue;
        if (regions[i].id & BORDER_REG)
            continue;
        regions[i].remap = true;
    }
    let regIdGen = 0;
    for (let i = 0; i < nreg; i++) {
        if (!regions[i].remap)
            continue;
        const oldId = regions[i].id;
        const newId = ++regIdGen;
        for (let j = i; j < nreg; j++) {
            if (regions[j].id === oldId) {
                regions[j].id = newId;
                regions[j].remap = false;
            }
        }
    }
    compactHeightfield.maxRegions = regIdGen;
    // remap regions
    for (let i = 0; i < compactHeightfield.spanCount; i++) {
        if ((srcReg[i] & BORDER_REG) === 0) {
            srcReg[i] = regions[srcReg[i]].id;
        }
    }
    // return regions that we found to be overlapping
    for (let i = 0; i < nreg; i++) {
        if (regions[i].overlap) {
            overlaps.push(regions[i].id);
        }
    }
    return true;
};
/* helper functions for region merging and filtering */
const addUniqueFloorRegion = (reg, n) => {
    for (let i = 0; i < reg.floors.length; i++) {
        if (reg.floors[i] === n)
            return;
    }
    reg.floors.push(n);
};
const isRegionConnectedToBorder = (reg) => {
    for (let i = 0; i < reg.connections.length; i++) {
        if (reg.connections[i] === 0)
            return true;
    }
    return false;
};
const canMergeWithRegion = (rega, regb) => {
    if (rega.areaType !== regb.areaType)
        return false;
    let n = 0;
    for (let i = 0; i < rega.connections.length; i++) {
        if (rega.connections[i] === regb.id)
            n++;
    }
    if (n > 1)
        return false;
    for (let i = 0; i < rega.floors.length; i++) {
        if (rega.floors[i] === regb.id)
            return false;
    }
    return true;
};
const mergeRegions = (rega, regb) => {
    const aid = rega.id;
    const bid = regb.id;
    // duplicate current neighborhood
    const acon = Array.from(rega.connections);
    const bcon = regb.connections;
    // find insertion point on A
    let insa = -1;
    for (let i = 0; i < acon.length; i++) {
        if (acon[i] === bid) {
            insa = i;
            break;
        }
    }
    if (insa === -1)
        return false;
    // find insertion point on B
    let insb = -1;
    for (let i = 0; i < bcon.length; i++) {
        if (bcon[i] === aid) {
            insb = i;
            break;
        }
    }
    if (insb === -1)
        return false;
    // merge neighbors
    rega.connections = [];
    for (let i = 0; i < acon.length - 1; i++) {
        rega.connections.push(acon[(insa + 1 + i) % acon.length]);
    }
    for (let i = 0; i < bcon.length - 1; i++) {
        rega.connections.push(bcon[(insb + 1 + i) % bcon.length]);
    }
    removeAdjacentNeighbors(rega);
    for (let j = 0; j < regb.floors.length; j++) {
        addUniqueFloorRegion(rega, regb.floors[j]);
    }
    rega.spanCount += regb.spanCount;
    regb.spanCount = 0;
    regb.connections = [];
    return true;
};
const removeAdjacentNeighbors = (reg) => {
    // remove adjacent duplicates
    for (let i = 0; i < reg.connections.length && reg.connections.length > 1;) {
        const ni = (i + 1) % reg.connections.length;
        if (reg.connections[i] === reg.connections[ni]) {
            // remove duplicate
            for (let j = i; j < reg.connections.length - 1; j++) {
                reg.connections[j] = reg.connections[j + 1];
            }
            reg.connections.pop();
        }
        else {
            i++;
        }
    }
};
const replaceNeighbor = (reg, oldId, newId) => {
    let neiChanged = false;
    for (let i = 0; i < reg.connections.length; i++) {
        if (reg.connections[i] === oldId) {
            reg.connections[i] = newId;
            neiChanged = true;
        }
    }
    for (let i = 0; i < reg.floors.length; i++) {
        if (reg.floors[i] === oldId) {
            reg.floors[i] = newId;
        }
    }
    if (neiChanged) {
        removeAdjacentNeighbors(reg);
    }
};
const isSolidEdge = (compactHeightfield, srcReg, x, y, i, dir) => {
    const span = compactHeightfield.spans[i];
    let r = 0;
    if (getCon(span, dir) !== NOT_CONNECTED) {
        const ax = x + DIR_OFFSETS[dir][0];
        const ay = y + DIR_OFFSETS[dir][1];
        const ai = compactHeightfield.cells[ax + ay * compactHeightfield.width].index + getCon(span, dir);
        r = srcReg[ai];
    }
    if (r === srcReg[i])
        return false;
    return true;
};
const walkContour$1 = (x, y, i, dir, compactHeightfield, srcReg, cont) => {
    const startDir = dir;
    const starti = i;
    const ss = compactHeightfield.spans[i];
    let curReg = 0;
    if (getCon(ss, dir) !== NOT_CONNECTED) {
        const ax = x + DIR_OFFSETS[dir][0];
        const ay = y + DIR_OFFSETS[dir][1];
        const ai = compactHeightfield.cells[ax + ay * compactHeightfield.width].index + getCon(ss, dir);
        curReg = srcReg[ai];
    }
    cont.push(curReg);
    let iter = 0;
    let currentX = x;
    let currentY = y;
    let currentI = i;
    let currentDir = dir;
    while (++iter < 40000) {
        const s = compactHeightfield.spans[currentI];
        if (isSolidEdge(compactHeightfield, srcReg, currentX, currentY, currentI, currentDir)) {
            // choose the edge corner
            let r = 0;
            if (getCon(s, currentDir) !== NOT_CONNECTED) {
                const ax = currentX + DIR_OFFSETS[currentDir][0];
                const ay = currentY + DIR_OFFSETS[currentDir][1];
                const ai = compactHeightfield.cells[ax + ay * compactHeightfield.width].index + getCon(s, currentDir);
                r = srcReg[ai];
            }
            if (r !== curReg) {
                curReg = r;
                cont.push(curReg);
            }
            currentDir = (currentDir + 1) & 0x3; // rotate CW
        }
        else {
            let ni = -1;
            const nx = currentX + DIR_OFFSETS[currentDir][0];
            const ny = currentY + DIR_OFFSETS[currentDir][1];
            if (getCon(s, currentDir) !== NOT_CONNECTED) {
                const nc = compactHeightfield.cells[nx + ny * compactHeightfield.width];
                ni = nc.index + getCon(s, currentDir);
            }
            if (ni === -1) {
                // should not happen
                return;
            }
            currentX = nx;
            currentY = ny;
            currentI = ni;
            currentDir = (currentDir + 3) & 0x3; // rotate CCW
        }
        if (starti === currentI && startDir === currentDir) {
            break;
        }
    }
    // remove adjacent duplicates
    if (cont.length > 1) {
        for (let j = 0; j < cont.length;) {
            const nj = (j + 1) % cont.length;
            if (cont[j] === cont[nj]) {
                for (let k = j; k < cont.length - 1; k++) {
                    cont[k] = cont[k + 1];
                }
                cont.pop();
            }
            else {
                j++;
            }
        }
    }
};
const NULL_NEI = 0xffff;
/**
 * Build regions using monotone partitioning algorithm.
 * This is an alternative to the watershed-based buildRegions function.
 * Monotone partitioning creates regions by sweeping the heightfield and
 * does not generate overlapping regions.
 */
const buildRegionsMonotone = (compactHeightfield, borderSize, minRegionArea, mergeRegionArea) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    let id = 1;
    const srcReg = new Array(compactHeightfield.spanCount).fill(0);
    const nsweeps = Math.max(compactHeightfield.width, compactHeightfield.height);
    const sweeps = new Array(nsweeps);
    // initialize sweeps array
    for (let i = 0; i < nsweeps; i++) {
        sweeps[i] = { rid: 0, id: 0, ns: 0, nei: 0 };
    }
    // mark border regions
    if (borderSize > 0) {
        const bw = Math.min(w, borderSize);
        const bh = Math.min(h, borderSize);
        paintRectRegion(0, bw, 0, h, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
        paintRectRegion(w - bw, w, 0, h, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
        paintRectRegion(0, w, 0, bh, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
        paintRectRegion(0, w, h - bh, h, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
    }
    compactHeightfield.borderSize = borderSize;
    const prev = new Array(256);
    // sweep one line at a time
    for (let y = borderSize; y < h - borderSize; y++) {
        // collect spans from this row
        if (prev.length < id + 1) {
            prev.length = id + 1;
        }
        prev.fill(0, 0, id);
        let rid = 1;
        for (let x = borderSize; x < w - borderSize; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                const span = compactHeightfield.spans[i];
                if (compactHeightfield.areas[i] === NULL_AREA)
                    continue;
                // check -x direction
                let previd = 0;
                if (getCon(span, 0) !== NOT_CONNECTED) {
                    const ax = x + DIR_OFFSETS[0][0];
                    const ay = y + DIR_OFFSETS[0][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 0);
                    if ((srcReg[ai] & BORDER_REG) === 0 && compactHeightfield.areas[i] === compactHeightfield.areas[ai]) {
                        previd = srcReg[ai];
                    }
                }
                if (!previd) {
                    previd = rid++;
                    sweeps[previd].rid = previd;
                    sweeps[previd].ns = 0;
                    sweeps[previd].nei = 0;
                }
                // check -y direction
                if (getCon(span, 3) !== NOT_CONNECTED) {
                    const ax = x + DIR_OFFSETS[3][0];
                    const ay = y + DIR_OFFSETS[3][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 3);
                    if (srcReg[ai] &&
                        (srcReg[ai] & BORDER_REG) === 0 &&
                        compactHeightfield.areas[i] === compactHeightfield.areas[ai]) {
                        const nr = srcReg[ai];
                        if (!sweeps[previd].nei || sweeps[previd].nei === nr) {
                            sweeps[previd].nei = nr;
                            sweeps[previd].ns++;
                            prev[nr]++;
                        }
                        else {
                            sweeps[previd].nei = NULL_NEI;
                        }
                    }
                }
                srcReg[i] = previd;
            }
        }
        // create unique ID
        for (let i = 1; i < rid; i++) {
            if (sweeps[i].nei !== NULL_NEI && sweeps[i].nei !== 0 && prev[sweeps[i].nei] === sweeps[i].ns) {
                sweeps[i].id = sweeps[i].nei;
            }
            else {
                sweeps[i].id = id++;
            }
        }
        // remap IDs
        for (let x = borderSize; x < w - borderSize; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                if (srcReg[i] > 0 && srcReg[i] < rid) {
                    srcReg[i] = sweeps[srcReg[i]].id;
                }
            }
        }
    }
    // merge regions and filter out small regions
    const overlaps = [];
    compactHeightfield.maxRegions = id;
    if (!mergeAndFilterRegions(minRegionArea, mergeRegionArea, compactHeightfield, srcReg, overlaps)) ;
    // store the result
    for (let i = 0; i < compactHeightfield.spanCount; i++) {
        compactHeightfield.spans[i].region = srcReg[i];
    }
    return true;
};
/**
 * Add unique connection to region
 */
const addUniqueConnection = (reg, n) => {
    for (let i = 0; i < reg.connections.length; i++) {
        if (reg.connections[i] === n)
            return;
    }
    reg.connections.push(n);
};
/**
 * Merge and filter layer regions
 */
const mergeAndFilterLayerRegions = (minRegionArea, compactHeightfield, srcReg, maxRegionId) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    const nreg = maxRegionId.value + 1;
    // construct regions
    const regions = [];
    for (let i = 0; i < nreg; i++) {
        regions.push({
            spanCount: 0,
            id: i,
            areaType: 0,
            remap: false,
            visited: false,
            overlap: false,
            connectsToBorder: false,
            ymin: 0xffff,
            ymax: 0,
            connections: [],
            floors: [],
        });
    }
    // find region neighbours and overlapping regions
    const lregs = [];
    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            lregs.length = 0;
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                const span = compactHeightfield.spans[i];
                const area = compactHeightfield.areas[i];
                const ri = srcReg[i];
                if (ri === 0 || ri >= nreg)
                    continue;
                const reg = regions[ri];
                reg.spanCount++;
                reg.areaType = area;
                reg.ymin = Math.min(reg.ymin, span.y);
                reg.ymax = Math.max(reg.ymax, span.y);
                // collect all region layers
                lregs.push(ri);
                // update neighbours
                for (let dir = 0; dir < 4; dir++) {
                    if (getCon(span, dir) !== NOT_CONNECTED) {
                        const ax = x + DIR_OFFSETS[dir][0];
                        const ay = y + DIR_OFFSETS[dir][1];
                        const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
                        const rai = srcReg[ai];
                        if (rai > 0 && rai < nreg && rai !== ri) {
                            addUniqueConnection(reg, rai);
                        }
                        if (rai & BORDER_REG) {
                            reg.connectsToBorder = true;
                        }
                    }
                }
            }
            // update overlapping regions
            for (let i = 0; i < lregs.length - 1; i++) {
                for (let j = i + 1; j < lregs.length; j++) {
                    if (lregs[i] !== lregs[j]) {
                        const ri = regions[lregs[i]];
                        const rj = regions[lregs[j]];
                        addUniqueFloorRegion(ri, lregs[j]);
                        addUniqueFloorRegion(rj, lregs[i]);
                    }
                }
            }
        }
    }
    // create 2D layers from regions
    let layerId = 1;
    for (let i = 0; i < nreg; i++) {
        regions[i].id = 0;
    }
    // merge monotone regions to create non-overlapping areas
    const stack = [];
    for (let i = 1; i < nreg; i++) {
        const root = regions[i];
        // skip already visited
        if (root.id !== 0)
            continue;
        // start search
        root.id = layerId;
        stack.length = 0;
        stack.push(i);
        while (stack.length > 0) {
            // pop front
            const regIndex = stack.shift();
            const reg = regions[regIndex];
            const ncons = reg.connections.length;
            for (let j = 0; j < ncons; j++) {
                const nei = reg.connections[j];
                const regn = regions[nei];
                // skip already visited
                if (regn.id !== 0)
                    continue;
                // skip if different area type
                if (reg.areaType !== regn.areaType)
                    continue;
                // skip if the neighbour is overlapping root region
                let overlap = false;
                for (let k = 0; k < root.floors.length; k++) {
                    if (root.floors[k] === nei) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap)
                    continue;
                // deepen
                stack.push(nei);
                // mark layer id
                regn.id = layerId;
                // merge current layers to root
                for (let k = 0; k < regn.floors.length; k++) {
                    addUniqueFloorRegion(root, regn.floors[k]);
                }
                root.ymin = Math.min(root.ymin, regn.ymin);
                root.ymax = Math.max(root.ymax, regn.ymax);
                root.spanCount += regn.spanCount;
                regn.spanCount = 0;
                root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
            }
        }
        layerId++;
    }
    // remove small regions
    for (let i = 0; i < nreg; i++) {
        if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder) {
            const reg = regions[i].id;
            for (let j = 0; j < nreg; j++) {
                if (regions[j].id === reg) {
                    regions[j].id = 0;
                }
            }
        }
    }
    // compress region IDs
    for (let i = 0; i < nreg; i++) {
        regions[i].remap = false;
        if (regions[i].id === 0)
            continue;
        if (regions[i].id & BORDER_REG)
            continue;
        regions[i].remap = true;
    }
    let regIdGen = 0;
    for (let i = 0; i < nreg; i++) {
        if (!regions[i].remap)
            continue;
        const oldId = regions[i].id;
        const newId = ++regIdGen;
        for (let j = i; j < nreg; j++) {
            if (regions[j].id === oldId) {
                regions[j].id = newId;
                regions[j].remap = false;
            }
        }
    }
    maxRegionId.value = regIdGen;
    // remap regions
    for (let i = 0; i < compactHeightfield.spanCount; i++) {
        if ((srcReg[i] & BORDER_REG) === 0) {
            srcReg[i] = regions[srcReg[i]].id;
        }
    }
    return true;
};
/**
 * Build layer regions using sweep-line algorithm.
 * This creates regions that can be used for building navigation mesh layers.
 * Layer regions handle overlapping walkable areas by creating separate layers.
 */
const buildLayerRegions = (compactHeightfield, borderSize, minRegionArea) => {
    const w = compactHeightfield.width;
    const h = compactHeightfield.height;
    let id = 1;
    const srcReg = new Array(compactHeightfield.spanCount).fill(0);
    const nsweeps = Math.max(compactHeightfield.width, compactHeightfield.height);
    const sweeps = new Array(nsweeps);
    // initialize sweeps array
    for (let i = 0; i < nsweeps; i++) {
        sweeps[i] = { rid: 0, id: 0, ns: 0, nei: 0 };
    }
    // mark border regions
    if (borderSize > 0) {
        const bw = Math.min(w, borderSize);
        const bh = Math.min(h, borderSize);
        paintRectRegion(0, bw, 0, h, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
        paintRectRegion(w - bw, w, 0, h, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
        paintRectRegion(0, w, 0, bh, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
        paintRectRegion(0, w, h - bh, h, id | BORDER_REG, compactHeightfield, srcReg);
        id++;
    }
    compactHeightfield.borderSize = borderSize;
    const prev = new Array(256);
    // sweep one line at a time
    for (let y = borderSize; y < h - borderSize; y++) {
        // collect spans from this row
        if (prev.length < id + 1) {
            prev.length = id + 1;
        }
        prev.fill(0, 0, id);
        let rid = 1;
        for (let x = borderSize; x < w - borderSize; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                const span = compactHeightfield.spans[i];
                if (compactHeightfield.areas[i] === NULL_AREA)
                    continue;
                // check -x direction
                let previd = 0;
                if (getCon(span, 0) !== NOT_CONNECTED) {
                    const ax = x + DIR_OFFSETS[0][0];
                    const ay = y + DIR_OFFSETS[0][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 0);
                    if ((srcReg[ai] & BORDER_REG) === 0 && compactHeightfield.areas[i] === compactHeightfield.areas[ai]) {
                        previd = srcReg[ai];
                    }
                }
                if (!previd) {
                    previd = rid++;
                    sweeps[previd].rid = previd;
                    sweeps[previd].ns = 0;
                    sweeps[previd].nei = 0;
                }
                // check -y direction
                if (getCon(span, 3) !== NOT_CONNECTED) {
                    const ax = x + DIR_OFFSETS[3][0];
                    const ay = y + DIR_OFFSETS[3][1];
                    const ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 3);
                    if (srcReg[ai] &&
                        (srcReg[ai] & BORDER_REG) === 0 &&
                        compactHeightfield.areas[i] === compactHeightfield.areas[ai]) {
                        const nr = srcReg[ai];
                        if (!sweeps[previd].nei || sweeps[previd].nei === nr) {
                            sweeps[previd].nei = nr;
                            sweeps[previd].ns++;
                            prev[nr]++;
                        }
                        else {
                            sweeps[previd].nei = NULL_NEI;
                        }
                    }
                }
                srcReg[i] = previd;
            }
        }
        // create unique ID
        for (let i = 1; i < rid; i++) {
            if (sweeps[i].nei !== NULL_NEI && sweeps[i].nei !== 0 && prev[sweeps[i].nei] === sweeps[i].ns) {
                sweeps[i].id = sweeps[i].nei;
            }
            else {
                sweeps[i].id = id++;
            }
        }
        // remap IDs
        for (let x = borderSize; x < w - borderSize; x++) {
            const cell = compactHeightfield.cells[x + y * w];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                if (srcReg[i] > 0 && srcReg[i] < rid) {
                    srcReg[i] = sweeps[srcReg[i]].id;
                }
            }
        }
    }
    // merge monotone regions to layers and remove small regions
    compactHeightfield.maxRegions = id;
    const maxRegionIdRef = { value: compactHeightfield.maxRegions };
    if (!mergeAndFilterLayerRegions(minRegionArea, compactHeightfield, srcReg, maxRegionIdRef)) ;
    compactHeightfield.maxRegions = maxRegionIdRef.value;
    // store the result
    for (let i = 0; i < compactHeightfield.spanCount; i++) {
        compactHeightfield.spans[i].region = srcReg[i];
    }
    return true;
};

// Maximum number of iterations for contour walking to prevent infinite loops
const MAX_CONTOUR_WALK_ITERATIONS = 40000;
var ContourBuildFlags;
(function (ContourBuildFlags) {
    /** tessellate solid (impassable) edges during contour simplification */
    ContourBuildFlags[ContourBuildFlags["CONTOUR_TESS_WALL_EDGES"] = 1] = "CONTOUR_TESS_WALL_EDGES";
    /** tessellate edges between areas during contour simplification */
    ContourBuildFlags[ContourBuildFlags["CONTOUR_TESS_AREA_EDGES"] = 2] = "CONTOUR_TESS_AREA_EDGES";
})(ContourBuildFlags || (ContourBuildFlags = {}));
// Helper function to get corner height
const getCornerHeight = (x, y, i, dir, chf, isBorderVertex) => {
    const s = chf.spans[i];
    let ch = s.y;
    const dirp = (dir + 1) & 0x3;
    const regs = new Array(4).fill(0);
    // Combine region and area codes in order to prevent
    // border vertices which are in between two areas to be removed.
    regs[0] = chf.spans[i].region | (chf.areas[i] << 16);
    if (getCon(s, dir) !== NOT_CONNECTED) {
        const ax = x + getDirOffsetX(dir);
        const ay = y + getDirOffsetY(dir);
        const ai = chf.cells[ax + ay * chf.width].index + getCon(s, dir);
        const as = chf.spans[ai];
        ch = Math.max(ch, as.y);
        regs[1] = chf.spans[ai].region | (chf.areas[ai] << 16);
        if (getCon(as, dirp) !== NOT_CONNECTED) {
            const ax2 = ax + getDirOffsetX(dirp);
            const ay2 = ay + getDirOffsetY(dirp);
            const ai2 = chf.cells[ax2 + ay2 * chf.width].index + getCon(as, dirp);
            const as2 = chf.spans[ai2];
            ch = Math.max(ch, as2.y);
            regs[2] = chf.spans[ai2].region | (chf.areas[ai2] << 16);
        }
    }
    if (getCon(s, dirp) !== NOT_CONNECTED) {
        const ax = x + getDirOffsetX(dirp);
        const ay = y + getDirOffsetY(dirp);
        const ai = chf.cells[ax + ay * chf.width].index + getCon(s, dirp);
        const as = chf.spans[ai];
        ch = Math.max(ch, as.y);
        regs[3] = chf.spans[ai].region | (chf.areas[ai] << 16);
        if (getCon(as, dir) !== NOT_CONNECTED) {
            const ax2 = ax + getDirOffsetX(dir);
            const ay2 = ay + getDirOffsetY(dir);
            const ai2 = chf.cells[ax2 + ay2 * chf.width].index + getCon(as, dir);
            const as2 = chf.spans[ai2];
            ch = Math.max(ch, as2.y);
            regs[2] = chf.spans[ai2].region | (chf.areas[ai2] << 16);
        }
    }
    // Check if the vertex is special edge vertex, these vertices will be removed later.
    for (let j = 0; j < 4; ++j) {
        const a = j;
        const b = (j + 1) & 0x3;
        const c = (j + 2) & 0x3;
        const d = (j + 3) & 0x3;
        // The vertex is a border vertex there are two same exterior cells in a row,
        // followed by two interior cells and none of the regions are out of bounds.
        const twoSameExts = (regs[a] & regs[b] & BORDER_REG) !== 0 && regs[a] === regs[b];
        const twoInts = ((regs[c] | regs[d]) & BORDER_REG) === 0;
        const intsSameArea = regs[c] >> 16 === regs[d] >> 16;
        const noZeros = regs[a] !== 0 && regs[b] !== 0 && regs[c] !== 0 && regs[d] !== 0;
        if (twoSameExts && twoInts && intsSameArea && noZeros) {
            isBorderVertex.value = true;
            break;
        }
    }
    return ch;
};
// Helper function to walk contour
const walkContour = (x, y, i, chf, flags, points) => {
    // Choose the first non-connected edge
    let dir = 0;
    while ((flags[i] & (1 << dir)) === 0) {
        dir++;
    }
    const startDir = dir;
    const starti = i;
    const area = chf.areas[i];
    let iter = 0;
    let currentX = x;
    let currentY = y;
    let currentI = i;
    while (++iter < MAX_CONTOUR_WALK_ITERATIONS) {
        if (flags[currentI] & (1 << dir)) {
            // Choose the edge corner
            const isBorderVertex = { value: false };
            let isAreaBorder = false;
            let px = currentX;
            const py = getCornerHeight(currentX, currentY, currentI, dir, chf, isBorderVertex);
            let pz = currentY;
            switch (dir) {
                case 0:
                    pz++;
                    break;
                case 1:
                    px++;
                    pz++;
                    break;
                case 2:
                    px++;
                    break;
            }
            let r = 0;
            const s = chf.spans[currentI];
            if (getCon(s, dir) !== NOT_CONNECTED) {
                const ax = currentX + getDirOffsetX(dir);
                const ay = currentY + getDirOffsetY(dir);
                const ai = chf.cells[ax + ay * chf.width].index + getCon(s, dir);
                r = chf.spans[ai].region;
                if (area !== chf.areas[ai]) {
                    isAreaBorder = true;
                }
            }
            if (isBorderVertex.value) {
                r |= BORDER_VERTEX;
            }
            if (isAreaBorder) {
                r |= AREA_BORDER;
            }
            points.push(px);
            points.push(py);
            points.push(pz);
            points.push(r);
            flags[currentI] &= ~(1 << dir); // Remove visited edges
            dir = (dir + 1) & 0x3; // Rotate CW
        }
        else {
            let ni = -1;
            const nx = currentX + getDirOffsetX(dir);
            const ny = currentY + getDirOffsetY(dir);
            const s = chf.spans[currentI];
            if (getCon(s, dir) !== NOT_CONNECTED) {
                const nc = chf.cells[nx + ny * chf.width];
                ni = nc.index + getCon(s, dir);
            }
            if (ni === -1) {
                // Should not happen.
                return;
            }
            currentX = nx;
            currentY = ny;
            currentI = ni;
            dir = (dir + 3) & 0x3; // Rotate CCW
        }
        if (starti === currentI && startDir === dir) {
            break;
        }
    }
};
// Helper function to calculate distance from point to line segment
const distancePtSeg = (x, z, px, pz, qx, qz) => {
    const pqx = qx - px;
    const pqz = qz - pz;
    const dx = x - px;
    const dz = z - pz;
    const d = pqx * pqx + pqz * pqz;
    let t = pqx * dx + pqz * dz;
    if (d > 0) {
        t /= d;
    }
    if (t < 0) {
        t = 0;
    }
    else if (t > 1) {
        t = 1;
    }
    const finalDx = px + t * pqx - x;
    const finalDz = pz + t * pqz - z;
    return finalDx * finalDx + finalDz * finalDz;
};
// Helper function to simplify contour
const simplifyContour = (points, simplified, maxError, maxEdgeLen, buildFlags) => {
    // Add initial points.
    let hasConnections = false;
    for (let i = 0; i < points.length; i += 4) {
        if ((points[i + 3] & CONTOUR_REG_MASK) !== 0) {
            hasConnections = true;
            break;
        }
    }
    if (hasConnections) {
        // The contour has some portals to other regions.
        // Add a new point to every location where the region changes.
        for (let i = 0, ni = Math.floor(points.length / 4); i < ni; ++i) {
            const ii = (i + 1) % ni;
            const differentRegs = (points[i * 4 + 3] & CONTOUR_REG_MASK) !== (points[ii * 4 + 3] & CONTOUR_REG_MASK);
            const areaBorders = (points[i * 4 + 3] & AREA_BORDER) !== (points[ii * 4 + 3] & AREA_BORDER);
            if (differentRegs || areaBorders) {
                simplified.push(points[i * 4 + 0]);
                simplified.push(points[i * 4 + 1]);
                simplified.push(points[i * 4 + 2]);
                simplified.push(i);
            }
        }
    }
    if (simplified.length === 0) {
        // If there is no connections at all,
        // create some initial points for the simplification process.
        // Find lower-left and upper-right vertices of the contour.
        let llx = points[0];
        let lly = points[1];
        let llz = points[2];
        let lli = 0;
        let urx = points[0];
        let ury = points[1];
        let urz = points[2];
        let uri = 0;
        for (let i = 0; i < points.length; i += 4) {
            const x = points[i + 0];
            const y = points[i + 1];
            const z = points[i + 2];
            if (x < llx || (x === llx && z < llz)) {
                llx = x;
                lly = y;
                llz = z;
                lli = Math.floor(i / 4);
            }
            if (x > urx || (x === urx && z > urz)) {
                urx = x;
                ury = y;
                urz = z;
                uri = Math.floor(i / 4);
            }
        }
        simplified.push(llx);
        simplified.push(lly);
        simplified.push(llz);
        simplified.push(lli);
        simplified.push(urx);
        simplified.push(ury);
        simplified.push(urz);
        simplified.push(uri);
    }
    // Add points until all raw points are within
    // error tolerance to the simplified shape.
    const pn = Math.floor(points.length / 4);
    for (let i = 0; i < Math.floor(simplified.length / 4);) {
        const ii = (i + 1) % Math.floor(simplified.length / 4);
        const ax = simplified[i * 4 + 0];
        const az = simplified[i * 4 + 2];
        const ai = simplified[i * 4 + 3];
        const bx = simplified[ii * 4 + 0];
        const bz = simplified[ii * 4 + 2];
        const bi = simplified[ii * 4 + 3];
        // Find maximum deviation from the segment.
        let maxd = 0;
        let maxi = -1;
        let ci;
        let cinc;
        let endi;
        // Traverse the segment in lexilogical order so that the
        // max deviation is calculated similarly when traversing
        // opposite segments.
        let segAx = ax;
        let segAz = az;
        let segBx = bx;
        let segBz = bz;
        if (bx > ax || (bx === ax && bz > az)) {
            cinc = 1;
            ci = (ai + cinc) % pn;
            endi = bi;
        }
        else {
            cinc = pn - 1;
            ci = (bi + cinc) % pn;
            endi = ai;
            // Swap ax, bx and az, bz
            segAx = bx;
            segBx = ax;
            segAz = bz;
            segBz = az;
        }
        // Tessellate only outer edges or edges between areas.
        if ((points[ci * 4 + 3] & CONTOUR_REG_MASK) === 0 || points[ci * 4 + 3] & AREA_BORDER) {
            while (ci !== endi) {
                const d = distancePtSeg(points[ci * 4 + 0], points[ci * 4 + 2], segAx, segAz, segBx, segBz);
                if (d > maxd) {
                    maxd = d;
                    maxi = ci;
                }
                ci = (ci + cinc) % pn;
            }
        }
        // If the max deviation is larger than accepted error,
        // add new point, else continue to next segment.
        if (maxi !== -1 && maxd > maxError * maxError) {
            // Add space for the new point.
            const oldLength = simplified.length;
            simplified.length = oldLength + 4;
            const n = Math.floor(simplified.length / 4);
            for (let j = n - 1; j > i; --j) {
                simplified[j * 4 + 0] = simplified[(j - 1) * 4 + 0];
                simplified[j * 4 + 1] = simplified[(j - 1) * 4 + 1];
                simplified[j * 4 + 2] = simplified[(j - 1) * 4 + 2];
                simplified[j * 4 + 3] = simplified[(j - 1) * 4 + 3];
            }
            // Add the point.
            simplified[(i + 1) * 4 + 0] = points[maxi * 4 + 0];
            simplified[(i + 1) * 4 + 1] = points[maxi * 4 + 1];
            simplified[(i + 1) * 4 + 2] = points[maxi * 4 + 2];
            simplified[(i + 1) * 4 + 3] = maxi;
        }
        else {
            ++i;
        }
    }
    // Split too long edges.
    if (maxEdgeLen > 0 &&
        (buildFlags & (ContourBuildFlags.CONTOUR_TESS_WALL_EDGES | ContourBuildFlags.CONTOUR_TESS_AREA_EDGES)) !== 0) {
        for (let i = 0; i < Math.floor(simplified.length / 4);) {
            const ii = (i + 1) % Math.floor(simplified.length / 4);
            const ax = simplified[i * 4 + 0];
            const az = simplified[i * 4 + 2];
            const ai = simplified[i * 4 + 3];
            const bx = simplified[ii * 4 + 0];
            const bz = simplified[ii * 4 + 2];
            const bi = simplified[ii * 4 + 3];
            // Find maximum deviation from the segment.
            let maxi = -1;
            const ci = (ai + 1) % pn;
            // Tessellate only outer edges or edges between areas.
            let tess = false;
            // Wall edges.
            if (buildFlags & ContourBuildFlags.CONTOUR_TESS_WALL_EDGES && (points[ci * 4 + 3] & CONTOUR_REG_MASK) === 0) {
                tess = true;
            }
            // Edges between areas.
            if (buildFlags & ContourBuildFlags.CONTOUR_TESS_AREA_EDGES && points[ci * 4 + 3] & AREA_BORDER) {
                tess = true;
            }
            if (tess) {
                const dx = bx - ax;
                const dz = bz - az;
                if (dx * dx + dz * dz > maxEdgeLen * maxEdgeLen) {
                    // Round based on the segments in lexilogical order so that the
                    // max tesselation is consistent regardless in which direction
                    // segments are traversed.
                    const n = bi < ai ? bi + pn - ai : bi - ai;
                    if (n > 1) {
                        if (bx > ax || (bx === ax && bz > az)) {
                            maxi = (ai + Math.floor(n / 2)) % pn;
                        }
                        else {
                            maxi = (ai + Math.floor((n + 1) / 2)) % pn;
                        }
                    }
                }
            }
            // If an edge is too long, add a point to split it.
            if (maxi !== -1) {
                // Add space for the new point.
                const oldLength = simplified.length;
                simplified.length = oldLength + 4;
                const n = Math.floor(simplified.length / 4);
                for (let j = n - 1; j > i; --j) {
                    simplified[j * 4 + 0] = simplified[(j - 1) * 4 + 0];
                    simplified[j * 4 + 1] = simplified[(j - 1) * 4 + 1];
                    simplified[j * 4 + 2] = simplified[(j - 1) * 4 + 2];
                    simplified[j * 4 + 3] = simplified[(j - 1) * 4 + 3];
                }
                // Add the point.
                simplified[(i + 1) * 4 + 0] = points[maxi * 4 + 0];
                simplified[(i + 1) * 4 + 1] = points[maxi * 4 + 1];
                simplified[(i + 1) * 4 + 2] = points[maxi * 4 + 2];
                simplified[(i + 1) * 4 + 3] = maxi;
            }
            else {
                ++i;
            }
        }
    }
    for (let i = 0; i < Math.floor(simplified.length / 4); ++i) {
        // The edge vertex flag is take from the current raw point,
        // and the neighbour region is take from the next raw point.
        const ai = (simplified[i * 4 + 3] + 1) % pn;
        const bi = simplified[i * 4 + 3];
        simplified[i * 4 + 3] = (points[ai * 4 + 3] & (CONTOUR_REG_MASK | AREA_BORDER)) | (points[bi * 4 + 3] & BORDER_VERTEX);
    }
};
// Helper function to calculate area of polygon
const calcAreaOfPolygon2D = (verts, nverts) => {
    let area = 0;
    for (let i = 0, j = nverts - 1; i < nverts; j = i++) {
        const vi = i * 4;
        const vj = j * 4;
        area += verts[vi] * verts[vj + 2] - verts[vj] * verts[vi + 2];
    }
    return Math.floor((area + 1) / 2);
};
// Helper functions for polygon operations
const prev$2 = (i, n) => (i - 1 >= 0 ? i - 1 : n - 1);
const next$2 = (i, n) => (i + 1 < n ? i + 1 : 0);
const area2$1 = (a, b, c) => {
    return (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
};
// Returns true iff c is strictly to the left of the directed
// line through a to b.
const left$1 = (a, b, c) => {
    return area2$1(a, b, c) < 0;
};
const leftOn$1 = (a, b, c) => {
    return area2$1(a, b, c) <= 0;
};
const collinear$1 = (a, b, c) => {
    return area2$1(a, b, c) === 0;
};
//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
const intersectProp$1 = (a, b, c, d) => {
    // Eliminate improper cases.
    if (collinear$1(a, b, c) || collinear$1(a, b, d) || collinear$1(c, d, a) || collinear$1(c, d, b))
        return false;
    return xorb$1(left$1(a, b, c), left$1(a, b, d)) && xorb$1(left$1(c, d, a), left$1(c, d, b));
};
// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segment ab.
const between$1 = (a, b, c) => {
    if (!collinear$1(a, b, c))
        return false;
    // If ab not vertical, check betweenness on x; else on y.
    if (a[0] !== b[0])
        return (a[0] <= c[0] && c[0] <= b[0]) || (a[0] >= c[0] && c[0] >= b[0]);
    else
        return (a[1] <= c[1] && c[1] <= b[1]) || (a[1] >= c[1] && c[1] >= b[1]);
};
// Returns true iff segments ab and cd intersect, properly or improperly.
const intersect$1 = (a, b, c, d) => {
    if (intersectProp$1(a, b, c, d))
        return true;
    else if (between$1(a, b, c) || between$1(a, b, d) || between$1(c, d, a) || between$1(c, d, b))
        return true;
    else
        return false;
};
const _intersectSegContour_p0 = vec2.create();
const _intersectSegContour_p1 = vec2.create();
const intersectSegContour = (d0, d1, i, n, verts) => {
    // For each edge (k,k+1) of P
    for (let k = 0; k < n; k++) {
        const k1 = next$2(k, n);
        // Skip edges incident to i.
        if (i === k || i === k1) {
            continue;
        }
        const p0 = vec2.set(_intersectSegContour_p0, verts[k * 4], verts[k * 4 + 2]);
        const p1 = vec2.set(_intersectSegContour_p1, verts[k1 * 4], verts[k1 * 4 + 2]);
        if (vec2.equals(d0, p0) || vec2.equals(d1, p0) || vec2.equals(d0, p1) || vec2.equals(d1, p1)) {
            continue;
        }
        if (intersect$1(d0, d1, p0, p1)) {
            return true;
        }
    }
    return false;
};
const _inCone_pi = vec2.create();
const _inCone_pi1 = vec2.create();
const _inCone_pin1 = vec2.create();
const inCone$1 = (i, n, verts, pj) => {
    const piIdx = i * 4;
    const pi1Idx = next$2(i, n) * 4;
    const pin1Idx = prev$2(i, n) * 4;
    const pi = vec2.set(_inCone_pi, verts[piIdx], verts[piIdx + 2]);
    const pi1 = vec2.set(_inCone_pi1, verts[pi1Idx], verts[pi1Idx + 2]);
    const pin1 = vec2.set(_inCone_pin1, verts[pin1Idx], verts[pin1Idx + 2]);
    // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
    if (leftOn$1(pin1, pi, pi1)) {
        return left$1(pi, pj, pin1) && left$1(pj, pi, pi1);
    }
    // Assume (i-1,i,i+1) not collinear.
    // else P[i] is reflex.
    return !(leftOn$1(pi, pj, pi1) && leftOn$1(pj, pi, pin1));
};
const xorb$1 = (x, y) => {
    return !x !== !y;
};
const vequal = (verticesA, vertexAIdx, verticesB, vertexBIdx) => {
    const offsetA = vertexAIdx * 4;
    const offsetB = vertexBIdx * 4;
    return verticesA[offsetA] === verticesB[offsetB] && verticesA[offsetA + 2] === verticesB[offsetB + 2];
};
const removeDegenerateSegments = (simplified) => {
    // Remove adjacent vertices which are equal on xz-plane,
    // or else the triangulator will get confused.
    // Iterate backwards to avoid index shifting issues when removing elements.
    let npts = Math.floor(simplified.length / 4);
    for (let i = npts - 1; i >= 0; --i) {
        const ni = next$2(i, npts);
        if (vequal(simplified, i, simplified, ni)) {
            // Degenerate segment, remove.
            for (let j = i; j < Math.floor(simplified.length / 4) - 1; ++j) {
                simplified[j * 4 + 0] = simplified[(j + 1) * 4 + 0];
                simplified[j * 4 + 1] = simplified[(j + 1) * 4 + 1];
                simplified[j * 4 + 2] = simplified[(j + 1) * 4 + 2];
                simplified[j * 4 + 3] = simplified[(j + 1) * 4 + 3];
            }
            simplified.splice(-4, 4);
            npts--;
        }
    }
};
const mergeContours = (ca, cb, ia, ib) => {
    const maxVerts = ca.nVertices + cb.nVertices + 2;
    const verts = new Array(maxVerts * 4);
    let nv = 0;
    // Copy contour A.
    for (let i = 0; i <= ca.nVertices; ++i) {
        const srcIndex = ((ia + i) % ca.nVertices) * 4;
        verts[nv * 4 + 0] = ca.vertices[srcIndex + 0];
        verts[nv * 4 + 1] = ca.vertices[srcIndex + 1];
        verts[nv * 4 + 2] = ca.vertices[srcIndex + 2];
        verts[nv * 4 + 3] = ca.vertices[srcIndex + 3];
        nv++;
    }
    // Copy contour B
    for (let i = 0; i <= cb.nVertices; ++i) {
        const srcIndex = ((ib + i) % cb.nVertices) * 4;
        verts[nv * 4 + 0] = cb.vertices[srcIndex + 0];
        verts[nv * 4 + 1] = cb.vertices[srcIndex + 1];
        verts[nv * 4 + 2] = cb.vertices[srcIndex + 2];
        verts[nv * 4 + 3] = cb.vertices[srcIndex + 3];
        nv++;
    }
    ca.vertices = verts;
    ca.nVertices = nv;
    cb.vertices = [];
    cb.nVertices = 0;
    return true;
};
// Finds the lowest leftmost vertex of a contour.
const findLeftMostVertex = (contour) => {
    let minx = contour.vertices[0];
    let minz = contour.vertices[2];
    let leftmost = 0;
    for (let i = 1; i < contour.nVertices; i++) {
        const x = contour.vertices[i * 4 + 0];
        const z = contour.vertices[i * 4 + 2];
        if (x < minx || (x === minx && z < minz)) {
            minx = x;
            minz = z;
            leftmost = i;
        }
    }
    return { minx, minz, leftmost };
};
const compareHoles = (a, b) => {
    if (a.minx === b.minx) {
        if (a.minz < b.minz) {
            return -1;
        }
        if (a.minz > b.minz) {
            return 1;
        }
    }
    else {
        if (a.minx < b.minx) {
            return -1;
        }
        if (a.minx > b.minx) {
            return 1;
        }
    }
    return 0;
};
const _mergeRegionHoles_corner = vec2.create();
const _mergeRegionHoles_pt = vec2.create();
const mergeRegionHoles = (ctx, region) => {
    // Sort holes from left to right.
    for (let i = 0; i < region.holes.length; i++) {
        const result = findLeftMostVertex(region.holes[i].contour);
        region.holes[i].minx = result.minx;
        region.holes[i].minz = result.minz;
        region.holes[i].leftmost = result.leftmost;
    }
    region.holes.sort(compareHoles);
    let maxVerts = region.outline.nVertices;
    for (let i = 0; i < region.holes.length; i++) {
        maxVerts += region.holes[i].contour.nVertices;
    }
    const diags = new Array(maxVerts);
    for (let i = 0; i < maxVerts; i++) {
        diags[i] = { vert: 0, dist: 0 };
    }
    const outline = region.outline;
    // Merge holes into the outline one by one.
    for (let i = 0; i < region.holes.length; i++) {
        const hole = region.holes[i].contour;
        let index = -1;
        let bestVertex = region.holes[i].leftmost;
        for (let iter = 0; iter < hole.nVertices; iter++) {
            // Find potential diagonals.
            // The 'best' vertex must be in the cone described by 3 consecutive vertices of the outline.
            // ..o j-1
            //   |
            //   |   * best
            //   |
            // j o-----o j+1
            //         :
            let ndiags = 0;
            const corner = vec2.set(_mergeRegionHoles_corner, hole.vertices[bestVertex * 4 + 0], hole.vertices[bestVertex * 4 + 2]);
            for (let j = 0; j < outline.nVertices; j++) {
                if (inCone$1(j, outline.nVertices, outline.vertices, corner)) {
                    const dx = outline.vertices[j * 4 + 0] - corner[0];
                    const dz = outline.vertices[j * 4 + 2] - corner[1];
                    diags[ndiags].vert = j;
                    diags[ndiags].dist = dx * dx + dz * dz;
                    ndiags++;
                }
            }
            // Sort potential diagonals by distance, we want to make the connection as short as possible.
            if (ndiags > 1) {
                // In-place sort of the first ndiags elements
                for (let a = 0; a < ndiags - 1; a++) {
                    for (let b = a + 1; b < ndiags; b++) {
                        if (diags[a].dist > diags[b].dist) {
                            const temp = diags[a];
                            diags[a] = diags[b];
                            diags[b] = temp;
                        }
                    }
                }
            }
            // Find a diagonal that is not intersecting the outline not the remaining holes.
            index = -1;
            for (let j = 0; j < ndiags; j++) {
                const ptIdx = diags[j].vert * 4;
                const pt = vec2.set(_mergeRegionHoles_pt, outline.vertices[ptIdx], outline.vertices[ptIdx + 2]);
                // TODO: should intersectSegContour be passed `diags[j].vert` instead of `diags[i].vert` ?
                let intersect = intersectSegContour(pt, corner, diags[i].vert, outline.nVertices, outline.vertices);
                for (let k = i; k < region.holes.length && !intersect; k++) {
                    intersect =
                        intersect ||
                            intersectSegContour(pt, corner, -1, region.holes[k].contour.nVertices, region.holes[k].contour.vertices);
                }
                if (!intersect) {
                    index = diags[j].vert;
                    break;
                }
            }
            // If found non-intersecting diagonal, stop looking.
            if (index !== -1) {
                break;
            }
            // All the potential diagonals for the current vertex were intersecting, try next vertex.
            bestVertex = (bestVertex + 1) % hole.nVertices;
        }
        if (index === -1) {
            BuildContext.warn(ctx, 'mergeHoles: Failed to find merge points for outline and hole.');
            continue;
        }
        if (!mergeContours(region.outline, hole, index, bestVertex)) ;
    }
};
const buildContours = (ctx, compactHeightfield, maxSimplificationError, maxEdgeLength, buildFlags) => {
    const width = compactHeightfield.width;
    const height = compactHeightfield.height;
    const borderSize = compactHeightfield.borderSize;
    // Initialize contour set
    const contourSet = {
        contours: [],
        bounds: box3.clone(compactHeightfield.bounds),
        cellSize: compactHeightfield.cellSize,
        cellHeight: compactHeightfield.cellHeight,
        width: compactHeightfield.width - compactHeightfield.borderSize * 2,
        height: compactHeightfield.height - compactHeightfield.borderSize * 2,
        borderSize: compactHeightfield.borderSize,
        maxError: maxSimplificationError,
    };
    // If the heightfield was build with borderSize, remove the offset.
    if (borderSize > 0) {
        const pad = borderSize * compactHeightfield.cellSize;
        contourSet.bounds[0][0] += pad;
        contourSet.bounds[0][2] += pad;
        contourSet.bounds[1][0] -= pad;
        contourSet.bounds[1][2] -= pad;
    }
    const flags = new Array(compactHeightfield.spanCount).fill(0);
    // Mark boundaries.
    for (let y = 0; y < height; ++y) {
        for (let x = 0; x < width; ++x) {
            const c = compactHeightfield.cells[x + y * width];
            for (let i = c.index; i < c.index + c.count; ++i) {
                let res = 0;
                const s = compactHeightfield.spans[i];
                if (!compactHeightfield.spans[i].region || compactHeightfield.spans[i].region & BORDER_REG) {
                    flags[i] = 0;
                    continue;
                }
                for (let dir = 0; dir < 4; ++dir) {
                    let r = 0;
                    if (getCon(s, dir) !== NOT_CONNECTED) {
                        const ax = x + getDirOffsetX(dir);
                        const ay = y + getDirOffsetY(dir);
                        const ai = compactHeightfield.cells[ax + ay * width].index + getCon(s, dir);
                        r = compactHeightfield.spans[ai].region;
                    }
                    if (r === compactHeightfield.spans[i].region) {
                        res |= 1 << dir;
                    }
                }
                flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
            }
        }
    }
    const verts = [];
    const simplified = [];
    for (let y = 0; y < height; ++y) {
        for (let x = 0; x < width; ++x) {
            const c = compactHeightfield.cells[x + y * width];
            for (let i = c.index; i < c.index + c.count; ++i) {
                if (flags[i] === 0 || flags[i] === 0xf) {
                    flags[i] = 0;
                    continue;
                }
                const region = compactHeightfield.spans[i].region;
                if (!region || region & BORDER_REG) {
                    continue;
                }
                const area = compactHeightfield.areas[i];
                verts.length = 0;
                simplified.length = 0;
                walkContour(x, y, i, compactHeightfield, flags, verts);
                simplifyContour(verts, simplified, maxSimplificationError, maxEdgeLength, buildFlags);
                removeDegenerateSegments(simplified);
                // Create contour.
                if (Math.floor(simplified.length / 4) >= 3) {
                    const contour = {
                        nVertices: Math.floor(simplified.length / 4),
                        vertices: simplified.slice(),
                        nRawVertices: Math.floor(verts.length / 4),
                        rawVertices: verts.slice(),
                        reg: region,
                        area: area,
                    };
                    if (borderSize > 0) {
                        // If the heightfield was build with bordersize, remove the offset.
                        for (let j = 0; j < contour.nVertices; ++j) {
                            contour.vertices[j * 4 + 0] -= borderSize;
                            contour.vertices[j * 4 + 2] -= borderSize;
                        }
                        for (let j = 0; j < contour.nRawVertices; ++j) {
                            contour.rawVertices[j * 4 + 0] -= borderSize;
                            contour.rawVertices[j * 4 + 2] -= borderSize;
                        }
                    }
                    contourSet.contours.push(contour);
                }
            }
        }
    }
    // Merge holes if needed.
    if (contourSet.contours.length > 0) {
        // Calculate winding of all polygons.
        const winding = new Array(contourSet.contours.length);
        let nholes = 0;
        for (let i = 0; i < contourSet.contours.length; ++i) {
            const contour = contourSet.contours[i];
            // If the contour is wound backwards, it is a hole.
            winding[i] = calcAreaOfPolygon2D(contour.vertices, contour.nVertices) < 0 ? -1 : 1;
            if (winding[i] < 0) {
                nholes++;
            }
        }
        if (nholes > 0) {
            // Collect outline contour and holes contours per region.
            // We assume that there is one outline and multiple holes.
            const nregions = compactHeightfield.maxRegions + 1;
            const regions = new Array(nregions);
            for (let i = 0; i < nregions; i++) {
                regions[i] = {
                    outline: null,
                    holes: [],
                };
            }
            const holes = new Array(contourSet.contours.length);
            for (let i = 0; i < contourSet.contours.length; i++) {
                holes[i] = {
                    contour: contourSet.contours[i],
                    minx: 0,
                    minz: 0,
                    leftmost: 0,
                };
            }
            for (let i = 0; i < contourSet.contours.length; ++i) {
                const contour = contourSet.contours[i];
                const region = regions[contour.reg];
                // Positively wound contours are outlines, negative holes.
                if (winding[i] > 0) {
                    if (region.outline) {
                        BuildContext.error(ctx, `buildContours: Multiple outlines for region ${contour.reg}.`);
                    }
                    region.outline = contour;
                }
                else {
                    region.holes.push(holes[i]);
                }
            }
            // Finally merge each regions holes into the outline.
            for (let i = 0; i < nregions; i++) {
                const region = regions[i];
                if (region.holes.length === 0)
                    continue;
                if (region.outline) {
                    mergeRegionHoles(ctx, region);
                }
                else {
                    // The region does not have an outline.
                    // This can happen if the contour becaomes selfoverlapping because of
                    // too aggressive simplification settings.
                    BuildContext.error(ctx, `buildContours: Bad outline for region ${i}, contour simplification is likely too aggressive.`);
                }
            }
        }
    }
    return contourSet;
};

const SPAN_MAX_HEIGHT = 0x1fff; // 8191
const MAX_HEIGHTFIELD_HEIGHT = 0xffff;
const calculateGridSize = (outGridSize, bounds, cellSize) => {
    const minBounds = bounds[0];
    const maxBounds = bounds[1];
    outGridSize[0] = Math.floor((maxBounds[0] - minBounds[0]) / cellSize + 0.5);
    outGridSize[1] = Math.floor((maxBounds[2] - minBounds[2]) / cellSize + 0.5);
    return outGridSize;
};
const createHeightfield = (width, height, bounds, cellSize, cellHeight) => {
    const numSpans = width * height;
    const spans = new Array(numSpans).fill(null);
    return {
        width,
        height,
        spans,
        bounds,
        cellHeight,
        cellSize,
    };
};
/**
 * Adds a span to the heightfield. If the new span overlaps existing spans,
 * it will merge the new span with the existing ones.
 */
const addHeightfieldSpan = (heightfield, x, z, min, max, areaID, flagMergeThreshold) => {
    // Create the new span
    const newSpan = {
        min,
        max,
        area: areaID,
        next: null,
    };
    const columnIndex = x + z * heightfield.width;
    let previousSpan = null;
    let currentSpan = heightfield.spans[columnIndex];
    // Insert the new span, possibly merging it with existing spans
    while (currentSpan !== null) {
        if (currentSpan.min > newSpan.max) {
            // Current span is completely after the new span, break
            break;
        }
        if (currentSpan.max < newSpan.min) {
            // Current span is completely before the new span. Keep going
            previousSpan = currentSpan;
            currentSpan = currentSpan.next;
        }
        else {
            // The new span overlaps with an existing span. Merge them
            if (currentSpan.min < newSpan.min) {
                newSpan.min = currentSpan.min;
            }
            if (currentSpan.max > newSpan.max) {
                newSpan.max = currentSpan.max;
            }
            // Merge flags
            if (Math.abs(newSpan.max - currentSpan.max) <= flagMergeThreshold) {
                // Higher area ID numbers indicate higher resolution priority
                newSpan.area = Math.max(newSpan.area, currentSpan.area);
            }
            // Remove the current span since it's now merged with newSpan
            const next = currentSpan.next;
            if (previousSpan) {
                previousSpan.next = next;
            }
            else {
                heightfield.spans[columnIndex] = next;
            }
            currentSpan = next;
        }
    }
    // Insert new span after prev
    if (previousSpan !== null) {
        newSpan.next = previousSpan.next;
        previousSpan.next = newSpan;
    }
    else {
        // This span should go before the others in the list
        newSpan.next = heightfield.spans[columnIndex];
        heightfield.spans[columnIndex] = newSpan;
    }
    return true;
};
/**
 * Divides a convex polygon of max 12 vertices into two convex polygons
 * across a separating axis.
 */
const dividePoly = (out, inVerts, inVertsCount, outVerts1, outVerts2, axisOffset, axis) => {
    // How far positive or negative away from the separating axis is each vertex
    const inVertAxisDelta = _inVertAxisDelta;
    for (let inVert = 0; inVert < inVertsCount; ++inVert) {
        inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis];
    }
    let poly1Vert = 0;
    let poly2Vert = 0;
    for (let inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA) {
        // If the two vertices are on the same side of the separating axis
        const sameSide = inVertAxisDelta[inVertA] >= 0 === inVertAxisDelta[inVertB] >= 0;
        if (!sameSide) {
            const s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
            outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
            outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
            outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
            // Copy to second polygon
            outVerts2[poly2Vert * 3 + 0] = outVerts1[poly1Vert * 3 + 0];
            outVerts2[poly2Vert * 3 + 1] = outVerts1[poly1Vert * 3 + 1];
            outVerts2[poly2Vert * 3 + 2] = outVerts1[poly1Vert * 3 + 2];
            poly1Vert++;
            poly2Vert++;
            // Add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
            // since these were already added above
            if (inVertAxisDelta[inVertA] > 0) {
                outVerts1[poly1Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
                outVerts1[poly1Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
                outVerts1[poly1Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
                poly1Vert++;
            }
            else if (inVertAxisDelta[inVertA] < 0) {
                outVerts2[poly2Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
                outVerts2[poly2Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
                outVerts2[poly2Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
                poly2Vert++;
            }
        }
        else {
            // Add the inVertA point to the right polygon. Addition is done even for points on the dividing line
            if (inVertAxisDelta[inVertA] >= 0) {
                outVerts1[poly1Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
                outVerts1[poly1Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
                outVerts1[poly1Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
                poly1Vert++;
                if (inVertAxisDelta[inVertA] !== 0) {
                    continue;
                }
            }
            outVerts2[poly2Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
            outVerts2[poly2Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
            outVerts2[poly2Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
            poly2Vert++;
        }
    }
    out.nv1 = poly1Vert;
    out.nv2 = poly2Vert;
};
const _triangleBounds = box3.create();
const _inVerts = new Array(7 * 3);
const _inRow = new Array(7 * 3);
const _p1 = new Array(7 * 3);
const _p2 = new Array(7 * 3);
const _inVertAxisDelta = new Array(12);
const _dividePolyResult = { nv1: 0, nv2: 0 };
const _v0$1 = vec3.create();
const _v1$1 = vec3.create();
const _v2$1 = vec3.create();
/**
 * Rasterize a single triangle to the heightfield
 */
const rasterizeTriangle = (v0, v1, v2, areaID, heightfield, flagMergeThreshold) => {
    // Calculate the bounding box of the triangle
    const triangleBounds = _triangleBounds;
    const triangleBoundsMin = triangleBounds[0];
    const triangleBoundsMax = triangleBounds[1];
    vec3.copy(_triangleBounds[0], v0);
    vec3.min(triangleBoundsMin, triangleBoundsMin, v1);
    vec3.min(triangleBoundsMin, triangleBoundsMin, v2);
    vec3.copy(triangleBoundsMax, v0);
    vec3.max(triangleBoundsMax, triangleBoundsMax, v1);
    vec3.max(triangleBoundsMax, triangleBoundsMax, v2);
    // If the triangle does not touch the bounding box of the heightfield, skip the triangle
    if (!box3.intersectsBox3(triangleBounds, heightfield.bounds)) {
        return true;
    }
    const heightfieldBoundsMin = heightfield.bounds[0];
    const heightfieldBoundsMax = heightfield.bounds[1];
    const w = heightfield.width;
    const h = heightfield.height;
    const by = heightfieldBoundsMax[1] - heightfieldBoundsMin[1];
    const cellSize = heightfield.cellSize;
    const cellHeight = heightfield.cellHeight;
    const inverseCellSize = 1.0 / cellSize;
    const inverseCellHeight = 1.0 / cellHeight;
    // Calculate the footprint of the triangle on the grid's z-axis
    let z0 = Math.floor((triangleBoundsMin[2] - heightfieldBoundsMin[2]) * inverseCellSize);
    let z1 = Math.floor((triangleBoundsMax[2] - heightfieldBoundsMin[2]) * inverseCellSize);
    // Use -1 rather than 0 to cut the polygon properly at the start of the tile
    z0 = clamp(z0, -1, h - 1);
    z1 = clamp(z1, 0, h - 1);
    // Clip the triangle into all grid cells it touches
    let inVerts = _inVerts;
    let inRow = _inRow;
    let p1 = _p1;
    let p2 = _p2;
    // Copy triangle vertices
    inVerts[0] = v0[0];
    inVerts[1] = v0[1];
    inVerts[2] = v0[2];
    inVerts[3] = v1[0];
    inVerts[4] = v1[1];
    inVerts[5] = v1[2];
    inVerts[6] = v2[0];
    inVerts[7] = v2[1];
    inVerts[8] = v2[2];
    let nvIn = 3;
    for (let z = z0; z <= z1; ++z) {
        // Clip polygon to row. Store the remaining polygon as well
        const cellZ = heightfieldBoundsMin[2] + z * cellSize;
        dividePoly(_dividePolyResult, inVerts, nvIn, inRow, p1, cellZ + cellSize, AXIS_Z);
        const nvRow = _dividePolyResult.nv1;
        const nvIn2 = _dividePolyResult.nv2;
        // Swap arrays
        const temp = inVerts;
        inVerts = p1;
        p1 = temp;
        nvIn = nvIn2;
        if (nvRow < 3) {
            continue;
        }
        if (z < 0) {
            continue;
        }
        // Find X-axis bounds of the row
        let minX = inRow[0];
        let maxX = inRow[0];
        for (let vert = 1; vert < nvRow; ++vert) {
            if (minX > inRow[vert * 3]) {
                minX = inRow[vert * 3];
            }
            if (maxX < inRow[vert * 3]) {
                maxX = inRow[vert * 3];
            }
        }
        let x0 = Math.floor((minX - heightfieldBoundsMin[0]) * inverseCellSize);
        let x1 = Math.floor((maxX - heightfieldBoundsMin[0]) * inverseCellSize);
        if (x1 < 0 || x0 >= w) {
            continue;
        }
        x0 = clamp(x0, -1, w - 1);
        x1 = clamp(x1, 0, w - 1);
        let nv2 = nvRow;
        for (let x = x0; x <= x1; ++x) {
            // Clip polygon to column. Store the remaining polygon as well
            const cx = heightfieldBoundsMin[0] + x * cellSize;
            dividePoly(_dividePolyResult, inRow, nv2, p1, p2, cx + cellSize, AXIS_X);
            const nv = _dividePolyResult.nv1;
            const nv2New = _dividePolyResult.nv2;
            // Swap arrays
            const temp = inRow;
            inRow = p2;
            p2 = temp;
            nv2 = nv2New;
            if (nv < 3) {
                continue;
            }
            if (x < 0) {
                continue;
            }
            // Calculate min and max of the span
            let spanMin = p1[1];
            let spanMax = p1[1];
            for (let vert = 1; vert < nv; ++vert) {
                spanMin = Math.min(spanMin, p1[vert * 3 + 1]);
                spanMax = Math.max(spanMax, p1[vert * 3 + 1]);
            }
            spanMin -= heightfieldBoundsMin[1];
            spanMax -= heightfieldBoundsMin[1];
            // Skip the span if it's completely outside the heightfield bounding box
            if (spanMax < 0.0) {
                continue;
            }
            if (spanMin > by) {
                continue;
            }
            // Clamp the span to the heightfield bounding box
            if (spanMin < 0.0) {
                spanMin = 0;
            }
            if (spanMax > by) {
                spanMax = by;
            }
            // Snap the span to the heightfield height grid
            const spanMinCellIndex = clamp(Math.floor(spanMin * inverseCellHeight), 0, SPAN_MAX_HEIGHT);
            const spanMaxCellIndex = clamp(Math.ceil(spanMax * inverseCellHeight), spanMinCellIndex + 1, SPAN_MAX_HEIGHT);
            if (!addHeightfieldSpan(heightfield, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold)) ;
        }
    }
    return true;
};
const rasterizeTriangles = (ctx, heightfield, vertices, indices, triAreaIds, flagMergeThreshold = 1) => {
    const numTris = indices.length / 3;
    for (let triIndex = 0; triIndex < numTris; ++triIndex) {
        const i0 = indices[triIndex * 3 + 0];
        const i1 = indices[triIndex * 3 + 1];
        const i2 = indices[triIndex * 3 + 2];
        const v0 = vec3.fromBuffer(_v0$1, vertices, i0 * 3);
        const v1 = vec3.fromBuffer(_v1$1, vertices, i1 * 3);
        const v2 = vec3.fromBuffer(_v2$1, vertices, i2 * 3);
        const areaId = triAreaIds[triIndex];
        if (!rasterizeTriangle(v0, v1, v2, areaId, heightfield, flagMergeThreshold)) {
            BuildContext.error(ctx, 'Failed to rasterize triangle');
            return false;
        }
    }
    return true;
};
const filterLowHangingWalkableObstacles = (heightfield, walkableClimb) => {
    const xSize = heightfield.width;
    const zSize = heightfield.height;
    for (let z = 0; z < zSize; ++z) {
        for (let x = 0; x < xSize; ++x) {
            let previousSpan = null;
            let previousWasWalkable = false;
            let previousAreaID = NULL_AREA;
            // For each span in the column...
            const columnIndex = x + z * xSize;
            let span = heightfield.spans[columnIndex];
            while (span !== null) {
                const walkable = span.area !== NULL_AREA;
                // If current span is not walkable, but there is walkable span just below it and the height difference
                // is small enough for the agent to walk over, mark the current span as walkable too.
                if (!walkable && previousWasWalkable && previousSpan && span.max - previousSpan.max <= walkableClimb) {
                    span.area = previousAreaID;
                }
                // Copy the original walkable value regardless of whether we changed it.
                // This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
                previousWasWalkable = walkable;
                previousAreaID = span.area;
                previousSpan = span;
                span = span.next;
            }
        }
    }
};
const filterLedgeSpans = (heightfield, walkableHeight, walkableClimb) => {
    const xSize = heightfield.width;
    const zSize = heightfield.height;
    // Mark spans that are adjacent to a ledge as unwalkable
    for (let z = 0; z < zSize; ++z) {
        for (let x = 0; x < xSize; ++x) {
            const columnIndex = x + z * xSize;
            let span = heightfield.spans[columnIndex];
            while (span !== null) {
                // Skip non-walkable spans
                if (span.area === NULL_AREA) {
                    span = span.next;
                    continue;
                }
                const floor = span.max;
                const ceiling = span.next ? span.next.min : MAX_HEIGHTFIELD_HEIGHT;
                // The difference between this walkable area and the lowest neighbor walkable area.
                // This is the difference between the current span and all neighbor spans that have
                // enough space for an agent to move between, but not accounting at all for surface slope.
                let lowestNeighborFloorDifference = MAX_HEIGHTFIELD_HEIGHT;
                // Min and max height of accessible neighbours.
                let lowestTraversableNeighborFloor = span.max;
                let highestTraversableNeighborFloor = span.max;
                for (let direction = 0; direction < 4; ++direction) {
                    const neighborX = x + getDirOffsetX(direction);
                    const neighborZ = z + getDirOffsetY(direction);
                    // Skip neighbours which are out of bounds.
                    if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize) {
                        lowestNeighborFloorDifference = -walkableClimb - 1;
                        break;
                    }
                    const neighborColumnIndex = neighborX + neighborZ * xSize;
                    let neighborSpan = heightfield.spans[neighborColumnIndex];
                    // The most we can step down to the neighbor is the walkableClimb distance.
                    // Start with the area under the neighbor span
                    let neighborCeiling = neighborSpan ? neighborSpan.min : MAX_HEIGHTFIELD_HEIGHT;
                    // Skip neighbour if the gap between the spans is too small.
                    if (Math.min(ceiling, neighborCeiling) - floor >= walkableHeight) {
                        lowestNeighborFloorDifference = -walkableClimb - 1;
                        break;
                    }
                    // For each span in the neighboring column...
                    while (neighborSpan !== null) {
                        const neighborFloor = neighborSpan.max;
                        neighborCeiling = neighborSpan.next ? neighborSpan.next.min : MAX_HEIGHTFIELD_HEIGHT;
                        // Only consider neighboring areas that have enough overlap to be potentially traversable.
                        if (Math.min(ceiling, neighborCeiling) - Math.max(floor, neighborFloor) < walkableHeight) {
                            // No space to traverse between them.
                            neighborSpan = neighborSpan.next;
                            continue;
                        }
                        const neighborFloorDifference = neighborFloor - floor;
                        lowestNeighborFloorDifference = Math.min(lowestNeighborFloorDifference, neighborFloorDifference);
                        // Find min/max accessible neighbor height.
                        // Only consider neighbors that are at most walkableClimb away.
                        if (Math.abs(neighborFloorDifference) <= walkableClimb) {
                            // There is space to move to the neighbor cell and the slope isn't too much.
                            lowestTraversableNeighborFloor = Math.min(lowestTraversableNeighborFloor, neighborFloor);
                            highestTraversableNeighborFloor = Math.max(highestTraversableNeighborFloor, neighborFloor);
                        }
                        else if (neighborFloorDifference < -walkableClimb) {
                            // We already know this will be considered a ledge span so we can early-out
                            break;
                        }
                        neighborSpan = neighborSpan.next;
                    }
                }
                // The current span is close to a ledge if the magnitude of the drop to any neighbour span is greater than the walkableClimb distance.
                // That is, there is a gap that is large enough to let an agent move between them, but the drop (surface slope) is too large to allow it.
                if (lowestNeighborFloorDifference < -walkableClimb) {
                    span.area = NULL_AREA;
                }
                // If the difference between all neighbor floors is too large, this is a steep slope, so mark the span as an unwalkable ledge.
                else if (highestTraversableNeighborFloor - lowestTraversableNeighborFloor > walkableClimb) {
                    span.area = NULL_AREA;
                }
                span = span.next;
            }
        }
    }
};
const filterWalkableLowHeightSpans = (heightfield, walkableHeight) => {
    const xSize = heightfield.width;
    const zSize = heightfield.height;
    // Remove walkable flag from spans which do not have enough
    // space above them for the agent to stand there.
    for (let z = 0; z < zSize; ++z) {
        for (let x = 0; x < xSize; ++x) {
            const columnIndex = x + z * xSize;
            let span = heightfield.spans[columnIndex];
            while (span !== null) {
                const floor = span.max;
                const ceiling = span.next ? span.next.min : MAX_HEIGHTFIELD_HEIGHT;
                if (ceiling - floor < walkableHeight) {
                    span.area = NULL_AREA;
                }
                span = span.next;
            }
        }
    }
};

const _edge0 = vec3.create();
const _edge1 = vec3.create();
/**
 * Calculates the normal vector of a triangle
 * @param inV0 First vertex [x, y, z]
 * @param inV1 Second vertex [x, y, z]
 * @param inV2 Third vertex [x, y, z]
 * @param outFaceNormal Output normal vector [x, y, z]
 */
const calcTriNormal = (inV0, inV1, inV2, outFaceNormal) => {
    // Calculate edge vectors: e0 = v1 - v0, e1 = v2 - v0
    vec3.subtract(_edge0, inV1, inV0);
    vec3.subtract(_edge1, inV2, inV0);
    // Calculate cross product: faceNormal = e0 × e1
    vec3.cross(outFaceNormal, _edge0, _edge1);
    // Normalize the result
    vec3.normalize(outFaceNormal, outFaceNormal);
};
const _triangleNormal = vec3.create();
const _v0 = vec3.create();
const _v1 = vec3.create();
const _v2 = vec3.create();
/**
 * Marks triangles as walkable based on their slope angle
 * @param inVertices Array of vertex coordinates [x0, y0, z0, x1, y1, z1, ...]
 * @param inIndices Array of triangle indices [i0, i1, i2, i3, i4, i5, ...]
 * @param outTriAreaIds Output array of triangle area IDs, with a length equal to inIndices.length / 3
 * @param walkableSlopeAngle Maximum walkable slope angle in degrees (default: 45)
 */
const markWalkableTriangles = (inVertices, inIndices, outTriAreaIds, walkableSlopeAngle = 45.0) => {
    // Convert walkable slope angle to threshold using cosine
    const walkableThr = Math.cos((walkableSlopeAngle / 180.0) * Math.PI);
    const numTris = inIndices.length / 3;
    for (let i = 0; i < numTris; ++i) {
        const triStartIndex = i * 3;
        const i0 = inIndices[triStartIndex];
        const i1 = inIndices[triStartIndex + 1];
        const i2 = inIndices[triStartIndex + 2];
        const v0 = vec3.fromBuffer(_v0, inVertices, i0 * 3);
        const v1 = vec3.fromBuffer(_v1, inVertices, i1 * 3);
        const v2 = vec3.fromBuffer(_v2, inVertices, i2 * 3);
        calcTriNormal(v0, v1, v2, _triangleNormal);
        if (_triangleNormal[1] > walkableThr) {
            outTriAreaIds[i] = WALKABLE_AREA;
        }
    }
};
/**
 * Clears (sets to NULL_AREA) triangles whose slope exceeds the walkable limit.
 * Mirrors markWalkableTriangles but does the inverse operation.
 * @param inVertices Array of vertex coordinates [x0, y0, z0, ...]
 * @param inIndices Array of triangle indices [i0, i1, i2, ...]
 * @param inOutTriAreaIds In/out array of triangle area IDs (modified in place)
 * @param walkableSlopeAngle Maximum walkable slope angle in degrees (default: 45)
 */
const clearUnwalkableTriangles = (inVertices, inIndices, inOutTriAreaIds, walkableSlopeAngle = 45.0) => {
    const walkableThr = Math.cos((walkableSlopeAngle / 180.0) * Math.PI);
    const numTris = inIndices.length / 3;
    for (let i = 0; i < numTris; ++i) {
        const triStartIndex = i * 3;
        const i0 = inIndices[triStartIndex];
        const i1 = inIndices[triStartIndex + 1];
        const i2 = inIndices[triStartIndex + 2];
        const v0 = vec3.fromBuffer(_v0, inVertices, i0 * 3);
        const v1 = vec3.fromBuffer(_v1, inVertices, i1 * 3);
        const v2 = vec3.fromBuffer(_v2, inVertices, i2 * 3);
        calcTriNormal(v0, v1, v2, _triangleNormal);
        if (_triangleNormal[1] <= walkableThr) {
            inOutTriAreaIds[i] = NULL_AREA;
        }
    }
};
const calculateMeshBounds = (outBounds, inVertices, inIndices) => {
    outBounds[0][0] = Number.POSITIVE_INFINITY;
    outBounds[0][1] = Number.POSITIVE_INFINITY;
    outBounds[0][2] = Number.POSITIVE_INFINITY;
    outBounds[1][0] = Number.NEGATIVE_INFINITY;
    outBounds[1][1] = Number.NEGATIVE_INFINITY;
    outBounds[1][2] = Number.NEGATIVE_INFINITY;
    const numTris = inIndices.length / 3;
    for (let i = 0; i < numTris; ++i) {
        const triStartIndex = i * 3;
        for (let j = 0; j < 3; ++j) {
            const index = inIndices[triStartIndex + j];
            const x = inVertices[index * 3];
            const y = inVertices[index * 3 + 1];
            const z = inVertices[index * 3 + 2];
            outBounds[0][0] = Math.min(outBounds[0][0], x);
            outBounds[0][1] = Math.min(outBounds[0][1], y);
            outBounds[0][2] = Math.min(outBounds[0][2], z);
            outBounds[1][0] = Math.max(outBounds[1][0], x);
            outBounds[1][1] = Math.max(outBounds[1][1], y);
            outBounds[1][2] = Math.max(outBounds[1][2], z);
        }
    }
    return outBounds;
};

const buildPolyNeighbours = (polys, vertices, borderSize, minX, minZ, maxX, maxZ) => {
    // initialize neis arrays for all polygons
    for (const poly of polys) {
        poly.neis = new Array(poly.vertices.length).fill(MESH_NULL_IDX);
    }
    // build adjacency information, finds internal neighbours for each polygon edge
    buildMeshAdjacency(polys, vertices.length / 3);
    // find portal edges
    if (borderSize > 0) {
        findPortalEdges(polys, vertices, minX, minZ, maxX, maxZ);
    }
    // final poly neis formatting
    finalizePolyNeighbours(polys);
};
/**
 * Finds polygon edge neighbours, populates the neis array for each polygon.
 */
const buildMeshAdjacency = (polys, vertexCount) => {
    const polygonCount = polys.length;
    const maxEdgeCount = polys.reduce((sum, poly) => sum + poly.vertices.length, 0);
    const firstEdge = new Array(vertexCount).fill(MESH_NULL_IDX);
    const nextEdge = new Array(maxEdgeCount).fill(MESH_NULL_IDX);
    let edgeCount = 0;
    const edges = [];
    for (let i = 0; i < vertexCount; i++) {
        firstEdge[i] = MESH_NULL_IDX;
    }
    // build edges
    for (let i = 0; i < polygonCount; i++) {
        const poly = polys[i];
        for (let j = 0; j < poly.vertices.length; j++) {
            const v0 = poly.vertices[j];
            const v1 = poly.vertices[(j + 1) % poly.vertices.length];
            if (v0 < v1) {
                const edge = {
                    vert: [v0, v1],
                    poly: [i, i],
                    polyEdge: [j, 0],
                };
                edges[edgeCount] = edge;
                nextEdge[edgeCount] = firstEdge[v0];
                firstEdge[v0] = edgeCount;
                edgeCount++;
            }
        }
    }
    // match edges
    for (let i = 0; i < polygonCount; i++) {
        const poly = polys[i];
        for (let j = 0; j < poly.vertices.length; j++) {
            const v0 = poly.vertices[j];
            const v1 = poly.vertices[(j + 1) % poly.vertices.length];
            if (v0 > v1) {
                for (let e = firstEdge[v1]; e !== MESH_NULL_IDX; e = nextEdge[e]) {
                    const edge = edges[e];
                    if (edge.vert[1] === v0 && edge.poly[0] === edge.poly[1]) {
                        edge.poly[1] = i;
                        edge.polyEdge[1] = j;
                        break;
                    }
                }
            }
        }
    }
    // store adjacency
    for (let i = 0; i < edgeCount; i++) {
        const e = edges[i];
        if (e.poly[0] !== e.poly[1]) {
            polys[e.poly[0]].neis[e.polyEdge[0]] = e.poly[1];
            polys[e.poly[1]].neis[e.polyEdge[1]] = e.poly[0];
        }
    }
};
const findPortalEdges = (polys, vertices, minX, minZ, maxX, maxZ) => {
    const va = [0, 0, 0];
    const vb = [0, 0, 0];
    for (let i = 0; i < polys.length; i++) {
        const poly = polys[i];
        for (let j = 0; j < poly.vertices.length; j++) {
            // skip connected edges
            if (poly.neis[j] !== MESH_NULL_IDX) {
                continue;
            }
            const nj = (j + 1) % poly.vertices.length;
            va[0] = vertices[poly.vertices[j] * 3];
            va[1] = vertices[poly.vertices[j] * 3 + 1];
            va[2] = vertices[poly.vertices[j] * 3 + 2];
            vb[0] = vertices[poly.vertices[nj] * 3];
            vb[1] = vertices[poly.vertices[nj] * 3 + 1];
            vb[2] = vertices[poly.vertices[nj] * 3 + 2];
            if (va[0] === minX && vb[0] === minX) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 0;
            }
            else if (va[2] === maxZ && vb[2] === maxZ) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 1;
            }
            else if (va[0] === maxX && vb[0] === maxX) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 2;
            }
            else if (va[2] === minZ && vb[2] === minZ) {
                poly.neis[j] = POLY_NEIS_FLAG_EXT_LINK | 3;
            }
        }
    }
};
const finalizePolyNeighbours = (polys) => {
    for (const poly of polys) {
        for (let i = 0; i < poly.neis.length; i++) {
            const neiValue = poly.neis[i];
            if (neiValue & POLY_NEIS_FLAG_EXT_LINK) {
                // border or portal edge
                const dir = neiValue & 0xf;
                if (dir === 0xf) {
                    poly.neis[i] = 0;
                }
                else if (dir === 0) {
                    poly.neis[i] = POLY_NEIS_FLAG_EXT_LINK | 4; // Portal x-
                }
                else if (dir === 1) {
                    poly.neis[i] = POLY_NEIS_FLAG_EXT_LINK | 2; // Portal z+
                }
                else if (dir === 2) {
                    poly.neis[i] = POLY_NEIS_FLAG_EXT_LINK | 0; // Portal x+
                }
                else if (dir === 3) {
                    poly.neis[i] = POLY_NEIS_FLAG_EXT_LINK | 6; // Portal z-
                }
                else {
                    // TODO: how to handle this case?
                    poly.neis[i] = 0;
                }
            }
            else {
                // normal internal connection (add 1 to convert from 0-based to 1-based indexing)
                poly.neis[i] = neiValue + 1;
            }
        }
    }
};

const polyMeshToTilePolys = (polyMesh) => {
    const vertices = polyMesh.vertices.slice();
    // create polys from input PolyMesh
    const nvp = polyMesh.maxVerticesPerPoly;
    const nPolys = polyMesh.nPolys;
    const polys = [];
    for (let i = 0; i < nPolys; i++) {
        const poly = {
            vertices: [],
            neis: [],
            flags: polyMesh.flags[i],
            area: polyMesh.areas[i],
        };
        // extract polygon data for this polygon
        const polyStart = i * nvp;
        for (let j = 0; j < nvp; j++) {
            const vertIndex = polyMesh.polys[polyStart + j];
            if (vertIndex === MESH_NULL_IDX)
                break;
            poly.vertices.push(vertIndex);
        }
        polys.push(poly);
    }
    // build poly neighbours information
    buildPolyNeighbours(polys, vertices, polyMesh.borderSize, 0, 0, polyMesh.localWidth, polyMesh.localHeight);
    // convert vertices to world space
    // we do this after buildPolyNeighbours so that neighbour calculation can be done with quantized values
    for (let i = 0; i < vertices.length; i += 3) {
        vertices[i] = polyMesh.bounds[0][0] + vertices[i] * polyMesh.cellSize;
        vertices[i + 1] = polyMesh.bounds[0][1] + vertices[i + 1] * polyMesh.cellHeight;
        vertices[i + 2] = polyMesh.bounds[0][2] + vertices[i + 2] * polyMesh.cellSize;
    }
    return {
        vertices,
        polys,
    };
};
/**
 * Builds NavMeshTile polys from given polygons. Use this method when you are creating a nav mesh tile from external polygon data.
 *
 * Use @see polyMeshToTilePolys if you need to convert a PolyMesh to NavMeshTile NavMeshPoly's.
 *
 * Computes poly neighbours used for internal polygon edge neighbour linking, and finds portal edges used for nav mesh tile stitching.
 * @param polygons polygons
 * @param vertices polygon vertices in world space
 * @param borderSize the border size. if above 0, portal edges will be marked
 * @param bounds the bounds of the polygon vertices
 * @returns NavMeshTile polygons
 */
const polygonsToNavMeshTilePolys = (polygons, vertices, borderSize, bounds) => {
    const polys = [];
    for (const poly of polygons) {
        polys.push({
            vertices: poly.vertices,
            flags: poly.flags,
            area: poly.area,
            neis: [],
        });
    }
    const minX = bounds[0][0];
    const minZ = bounds[0][2];
    const maxX = bounds[1][0];
    const maxZ = bounds[1][2];
    buildPolyNeighbours(polys, vertices, borderSize, minX, maxX, minZ, maxZ);
    return {
        vertices,
        polys,
    };
};
/**
 * Creates a detail mesh from the given polygon data using fan triangulation.
 * This is less precise than providing a detail mesh, but is acceptable for some use cases where accurate height data is not important.
 * @param polys
 * @returns
 */
const polysToTileDetailMesh = (polys) => {
    const detailTriangles = [];
    const detailMeshes = [];
    let tbase = 0;
    for (const polyId in polys) {
        const poly = polys[polyId];
        const nv = poly.vertices.length;
        // create detail mesh descriptor for this polygon
        const detailMesh = {
            verticesBase: 0, // no additional detail vertices when triangulating from polys
            verticesCount: 0, // no additional detail vertices when triangulating from polys
            trianglesBase: tbase, // starting triangle index
            trianglesCount: nv - 2, // number of triangles in fan triangulation
        };
        detailMeshes[polyId] = detailMesh;
        // triangulate polygon using fan triangulation (local indices within the polygon)
        for (let j = 2; j < nv; j++) {
            // create triangle using vertex 0 and two consecutive vertices
            detailTriangles.push(0); // first vertex (local index)
            detailTriangles.push(j - 1); // previous vertex (local index)
            detailTriangles.push(j); // current vertex (local index)
            // edge flags - bit for each edge that belongs to poly boundary
            let edgeFlags = 1 << 2; // edge 2 is always a polygon boundary
            if (j === 2)
                edgeFlags |= 1 << 0; // first triangle, edge 0 is boundary
            if (j === nv - 1)
                edgeFlags |= 1 << 4; // last triangle, edge 1 is boundary
            detailTriangles.push(edgeFlags);
            tbase++;
        }
    }
    return {
        detailMeshes,
        detailTriangles,
        detailVertices: [],
    };
};
/**
 * Converts a given PolyMeshDetail to the tile detail mesh format.
 * @param polys
 * @param polyMeshDetail
 * @returns
 */
const polyMeshDetailToTileDetailMesh = (polys, polyMeshDetail) => {
    const detailMeshes = [];
    const detailVertices = [];
    // store detail meshes and vertices.
    // the nav polygon vertices are stored as the first vertices on each mesh.
    // we compress the mesh data by skipping them and using the navmesh coordinates.
    let vbase = 0;
    for (let i = 0; i < polys.length; i++) {
        const poly = polys[i];
        const nPolyVertices = poly.vertices.length;
        const vb = polyMeshDetail.meshes[i * 4];
        const nDetailVertices = polyMeshDetail.meshes[i * 4 + 1];
        const trianglesBase = polyMeshDetail.meshes[i * 4 + 2];
        const trianglesCount = polyMeshDetail.meshes[i * 4 + 3];
        const nAdditionalDetailVertices = nDetailVertices - nPolyVertices;
        const detailMesh = {
            verticesBase: vbase,
            verticesCount: nAdditionalDetailVertices,
            trianglesBase: trianglesBase,
            trianglesCount: trianglesCount,
        };
        detailMeshes[i] = detailMesh;
        // Copy vertices except the first 'nv' verts which are equal to nav poly verts.
        if (nAdditionalDetailVertices > 0) {
            for (let j = nPolyVertices; j < nDetailVertices; j++) {
                const detailVertIndex = (vb + j) * 3;
                detailVertices.push(polyMeshDetail.vertices[detailVertIndex], polyMeshDetail.vertices[detailVertIndex + 1], polyMeshDetail.vertices[detailVertIndex + 2]);
            }
            vbase += nAdditionalDetailVertices;
        }
    }
    return {
        detailMeshes,
        detailVertices,
        detailTriangles: polyMeshDetail.triangles,
    };
};

const VERTEX_Y_TOLERANCE = 2;
const addVertex = (x, y, z, vertices, vflags, vertexMap) => {
    // key by X and Z only; we'll search the bucket for a vertex with similar Y
    const keyXZ = `${x},${z}`;
    const bucket = vertexMap[keyXZ];
    if (bucket) {
        for (let i = 0; i < bucket.length; i++) {
            const entry = bucket[i];
            const [entryY, idx] = entry;
            // x and z match, check y tolerance
            if (Math.abs(entryY - y) <= VERTEX_Y_TOLERANCE) {
                return idx;
            }
        }
    }
    // could not find — create new
    const i = Math.floor(vertices.length / 3);
    vertices.push(x, y, z);
    const newEntry = [y, i];
    if (bucket) {
        bucket.push(newEntry);
    }
    else {
        vertexMap[keyXZ] = [newEntry];
    }
    vflags.push(0);
    return i;
};
const prev$1 = (i, n) => (i - 1 >= 0 ? i - 1 : n - 1);
const next$1 = (i, n) => (i + 1 < n ? i + 1 : 0);
const area2 = (vertexA, vertexB, vertexC) => {
    return (vertexB[0] - vertexA[0]) * (vertexC[2] - vertexA[2]) - (vertexC[0] - vertexA[0]) * (vertexB[2] - vertexA[2]);
};
const xorb = (x, y) => !x !== !y;
// returns true if c is strictly to the left of the directed
// line through a to b.
const left = (firstVertex, secondVertex, testVertex) => area2(firstVertex, secondVertex, testVertex) < 0;
const leftOn = (firstVertex, secondVertex, testVertex) => area2(firstVertex, secondVertex, testVertex) <= 0;
const collinear = (firstVertex, secondVertex, testVertex) => area2(firstVertex, secondVertex, testVertex) === 0;
const intersectProp = (segmentAStart, segmentAEnd, segmentBStart, segmentBEnd) => {
    if (collinear(segmentAStart, segmentAEnd, segmentBStart) ||
        collinear(segmentAStart, segmentAEnd, segmentBEnd) ||
        collinear(segmentBStart, segmentBEnd, segmentAStart) ||
        collinear(segmentBStart, segmentBEnd, segmentAEnd)) {
        return false;
    }
    return (xorb(left(segmentAStart, segmentAEnd, segmentBStart), left(segmentAStart, segmentAEnd, segmentBEnd)) &&
        xorb(left(segmentBStart, segmentBEnd, segmentAStart), left(segmentBStart, segmentBEnd, segmentAEnd)));
};
const between = (startVertex, endVertex, testVertex) => {
    if (!collinear(startVertex, endVertex, testVertex))
        return false;
    if (startVertex[0] !== endVertex[0]) {
        return ((startVertex[0] <= testVertex[0] && testVertex[0] <= endVertex[0]) ||
            (startVertex[0] >= testVertex[0] && testVertex[0] >= endVertex[0]));
    }
    return ((startVertex[2] <= testVertex[2] && testVertex[2] <= endVertex[2]) ||
        (startVertex[2] >= testVertex[2] && testVertex[2] >= endVertex[2]));
};
const intersect = (segmentAStart, segmentAEnd, segmentBStart, segmentBEnd) => {
    if (intersectProp(segmentAStart, segmentAEnd, segmentBStart, segmentBEnd))
        return true;
    return (between(segmentAStart, segmentAEnd, segmentBStart) ||
        between(segmentAStart, segmentAEnd, segmentBEnd) ||
        between(segmentBStart, segmentBEnd, segmentAStart) ||
        between(segmentBStart, segmentBEnd, segmentAEnd));
};
// returns whether the two vertices are equal in the XZ plane
const vec3EqualXZ = (vertexA, vertexB) => vertexA[0] === vertexB[0] && vertexA[2] === vertexB[2];
const _diagonalStart = vec3.create();
const _diagonalEnd = vec3.create();
const _edgeStart = vec3.create();
const _edgeEnd = vec3.create();
// returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
const diagonalie = (startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) => {
    const diagonalStart = vec3.fromBuffer(_diagonalStart, vertices, (vertexIndices[startVertexIdx] & 0x0fffffff) * 4);
    const diagonalEnd = vec3.fromBuffer(_diagonalEnd, vertices, (vertexIndices[endVertexIdx] & 0x0fffffff) * 4);
    for (let k = 0; k < polygonVertexCount; k++) {
        const k1 = next$1(k, polygonVertexCount);
        if (!(k === startVertexIdx || k1 === startVertexIdx || k === endVertexIdx || k1 === endVertexIdx)) {
            const edgeStart = vec3.fromBuffer(_edgeStart, vertices, (vertexIndices[k] & 0x0fffffff) * 4);
            const edgeEnd = vec3.fromBuffer(_edgeEnd, vertices, (vertexIndices[k1] & 0x0fffffff) * 4);
            if (vec3EqualXZ(diagonalStart, edgeStart) ||
                vec3EqualXZ(diagonalEnd, edgeStart) ||
                vec3EqualXZ(diagonalStart, edgeEnd) ||
                vec3EqualXZ(diagonalEnd, edgeEnd)) {
                continue;
            }
            if (intersect(diagonalStart, diagonalEnd, edgeStart, edgeEnd)) {
                return false;
            }
        }
    }
    return true;
};
const _coneVertex = vec3.create();
const _testVertex = vec3.create();
const _nextVertex = vec3.create();
const _prevVertex = vec3.create();
const inCone = (coneVertexIdx, testVertexIdx, polygonVertexCount, vertices, vertexIndices) => {
    const coneVertex = vec3.fromBuffer(_coneVertex, vertices, (vertexIndices[coneVertexIdx] & 0x0fffffff) * 4);
    const testVertex = vec3.fromBuffer(_testVertex, vertices, (vertexIndices[testVertexIdx] & 0x0fffffff) * 4);
    const nextVertex = vec3.fromBuffer(_nextVertex, vertices, (vertexIndices[next$1(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
    const prevVertex = vec3.fromBuffer(_prevVertex, vertices, (vertexIndices[prev$1(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
    if (leftOn(prevVertex, coneVertex, nextVertex)) {
        return left(coneVertex, testVertex, prevVertex) && left(testVertex, coneVertex, nextVertex);
    }
    return !(leftOn(coneVertex, testVertex, nextVertex) && leftOn(testVertex, coneVertex, prevVertex));
};
const diagonal = (startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) => {
    return (inCone(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) &&
        diagonalie(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices));
};
const diagonalieLoose = (startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) => {
    const diagonalStart = vec3.fromBuffer(_diagonalStart, vertices, (vertexIndices[startVertexIdx] & 0x0fffffff) * 4);
    const diagonalEnd = vec3.fromBuffer(_diagonalEnd, vertices, (vertexIndices[endVertexIdx] & 0x0fffffff) * 4);
    for (let k = 0; k < polygonVertexCount; k++) {
        const k1 = next$1(k, polygonVertexCount);
        if (!(k === startVertexIdx || k1 === startVertexIdx || k === endVertexIdx || k1 === endVertexIdx)) {
            const edgeStart = vec3.fromBuffer(_edgeStart, vertices, (vertexIndices[k] & 0x0fffffff) * 4);
            const edgeEnd = vec3.fromBuffer(_edgeEnd, vertices, (vertexIndices[k1] & 0x0fffffff) * 4);
            if (vec3EqualXZ(diagonalStart, edgeStart) ||
                vec3EqualXZ(diagonalEnd, edgeStart) ||
                vec3EqualXZ(diagonalStart, edgeEnd) ||
                vec3EqualXZ(diagonalEnd, edgeEnd)) {
                continue;
            }
            if (intersectProp(diagonalStart, diagonalEnd, edgeStart, edgeEnd)) {
                return false;
            }
        }
    }
    return true;
};
const inConeLoose = (coneVertexIdx, testVertexIdx, polygonVertexCount, vertices, vertexIndices) => {
    const coneVertex = vec3.fromBuffer(_coneVertex, vertices, (vertexIndices[coneVertexIdx] & 0x0fffffff) * 4);
    const testVertex = vec3.fromBuffer(_testVertex, vertices, (vertexIndices[testVertexIdx] & 0x0fffffff) * 4);
    const nextVertex = vec3.fromBuffer(_nextVertex, vertices, (vertexIndices[next$1(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
    const prevVertex = vec3.fromBuffer(_prevVertex, vertices, (vertexIndices[prev$1(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
    if (leftOn(prevVertex, coneVertex, nextVertex)) {
        return leftOn(coneVertex, testVertex, prevVertex) && leftOn(testVertex, coneVertex, nextVertex);
    }
    return !(leftOn(coneVertex, testVertex, nextVertex) && leftOn(testVertex, coneVertex, prevVertex));
};
const diagonalLoose = (startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) => {
    return (inConeLoose(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) &&
        diagonalieLoose(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices));
};
const _triangulateP0 = vec3.create();
const _triangulateP2 = vec3.create();
const triangulate = (polygonVertexCount, vertices, vertexIndices, triangleIndices) => {
    let ntris = 0;
    let dst = 0;
    // mark removable vertices
    for (let i = 0; i < polygonVertexCount; i++) {
        const i1 = next$1(i, polygonVertexCount);
        const i2 = next$1(i1, polygonVertexCount);
        if (diagonal(i, i2, polygonVertexCount, vertices, vertexIndices)) {
            vertexIndices[i1] |= 0x80000000;
        }
    }
    let nv = polygonVertexCount;
    while (nv > 3) {
        let minLen = -1;
        let mini = -1;
        for (let i = 0; i < nv; i++) {
            const i1 = next$1(i, nv);
            if (vertexIndices[i1] & 0x80000000) {
                const p0 = vec3.fromBuffer(_triangulateP0, vertices, (vertexIndices[i] & 0x0fffffff) * 4);
                const p2 = vec3.fromBuffer(_triangulateP2, vertices, (vertexIndices[next$1(i1, nv)] & 0x0fffffff) * 4);
                const dx = p2[0] - p0[0];
                const dy = p2[2] - p0[2];
                const len = dx * dx + dy * dy;
                if (minLen < 0 || len < minLen) {
                    minLen = len;
                    mini = i;
                }
            }
        }
        if (mini === -1) {
            // try loose diagonal test
            for (let i = 0; i < nv; i++) {
                const i1 = next$1(i, nv);
                const i2 = next$1(i1, nv);
                if (diagonalLoose(i, i2, nv, vertices, vertexIndices)) {
                    // using loose version would be better but simplified
                    const p0 = vec3.fromBuffer(_triangulateP0, vertices, (vertexIndices[i] & 0x0fffffff) * 4);
                    const p2 = vec3.fromBuffer(_triangulateP2, vertices, (vertexIndices[next$1(i2, nv)] & 0x0fffffff) * 4);
                    const dx = p2[0] - p0[0];
                    const dy = p2[2] - p0[2];
                    const len = dx * dx + dy * dy;
                    if (minLen < 0 || len < minLen) {
                        minLen = len;
                        mini = i;
                    }
                }
            }
            if (mini === -1) {
                return -ntris;
            }
        }
        const i = mini;
        let i1 = next$1(i, nv);
        const i2 = next$1(i1, nv);
        triangleIndices[dst++] = vertexIndices[i] & 0x0fffffff;
        triangleIndices[dst++] = vertexIndices[i1] & 0x0fffffff;
        triangleIndices[dst++] = vertexIndices[i2] & 0x0fffffff;
        ntris++;
        // remove vertex
        nv--;
        for (let k = i1; k < nv; k++) {
            vertexIndices[k] = vertexIndices[k + 1];
        }
        if (i1 >= nv)
            i1 = 0;
        const iPrev = prev$1(i1, nv);
        // update diagonal flags
        if (diagonal(prev$1(iPrev, nv), i1, nv, vertices, vertexIndices)) {
            vertexIndices[iPrev] |= 0x80000000;
        }
        else {
            vertexIndices[iPrev] &= 0x0fffffff;
        }
        if (diagonal(iPrev, next$1(i1, nv), nv, vertices, vertexIndices)) {
            vertexIndices[i1] |= 0x80000000;
        }
        else {
            vertexIndices[i1] &= 0x0fffffff;
        }
    }
    // final triangle
    triangleIndices[dst++] = vertexIndices[0] & 0x0fffffff;
    triangleIndices[dst++] = vertexIndices[1] & 0x0fffffff;
    triangleIndices[dst++] = vertexIndices[2] & 0x0fffffff;
    ntris++;
    return ntris;
};
const countPolyVerts = (polygons, polyStartIdx, maxVerticesPerPoly) => {
    for (let i = 0; i < maxVerticesPerPoly; i++) {
        if (polygons[polyStartIdx + i] === MESH_NULL_IDX)
            return i;
    }
    return maxVerticesPerPoly;
};
const uleft = (firstVertex, secondVertex, testVertex) => {
    return ((secondVertex[0] - firstVertex[0]) * (testVertex[2] - firstVertex[2]) -
        (testVertex[0] - firstVertex[0]) * (secondVertex[2] - firstVertex[2]) <
        0);
};
const getPolyMergeValue = (polygons, polyAStartIdx, polyBStartIdx, vertices, maxVerticesPerPoly) => {
    const numVertsA = countPolyVerts(polygons, polyAStartIdx, maxVerticesPerPoly);
    const numVertsB = countPolyVerts(polygons, polyBStartIdx, maxVerticesPerPoly);
    if (numVertsA + numVertsB - 2 > maxVerticesPerPoly) {
        return { value: -1, ea: -1, eb: -1 };
    }
    let ea = -1;
    let eb = -1;
    // check if polygons share an edge
    for (let i = 0; i < numVertsA; i++) {
        let va0 = polygons[polyAStartIdx + i];
        let va1 = polygons[polyAStartIdx + ((i + 1) % numVertsA)];
        if (va0 > va1)
            [va0, va1] = [va1, va0];
        for (let j = 0; j < numVertsB; j++) {
            let vb0 = polygons[polyBStartIdx + j];
            let vb1 = polygons[polyBStartIdx + ((j + 1) % numVertsB)];
            if (vb0 > vb1)
                [vb0, vb1] = [vb1, vb0];
            if (va0 === vb0 && va1 === vb1) {
                ea = i;
                eb = j;
                break;
            }
        }
    }
    if (ea === -1 || eb === -1) {
        return { value: -1, ea: -1, eb: -1 };
    }
    // check convexity
    const va = polygons[polyAStartIdx + ((ea + numVertsA - 1) % numVertsA)];
    const vb = polygons[polyAStartIdx + ea];
    const vc = polygons[polyBStartIdx + ((eb + 2) % numVertsB)];
    if (!uleft([vertices[va * 3], 0, vertices[va * 3 + 2]], [vertices[vb * 3], 0, vertices[vb * 3 + 2]], [vertices[vc * 3], 0, vertices[vc * 3 + 2]])) {
        return { value: -1, ea: -1, eb: -1 };
    }
    const va2 = polygons[polyBStartIdx + ((eb + numVertsB - 1) % numVertsB)];
    const vb2 = polygons[polyBStartIdx + eb];
    const vc2 = polygons[polyAStartIdx + ((ea + 2) % numVertsA)];
    if (!uleft([vertices[va2 * 3], 0, vertices[va2 * 3 + 2]], [vertices[vb2 * 3], 0, vertices[vb2 * 3 + 2]], [vertices[vc2 * 3], 0, vertices[vc2 * 3 + 2]])) {
        return { value: -1, ea: -1, eb: -1 };
    }
    const vaEdge = polygons[polyAStartIdx + ea];
    const vbEdge = polygons[polyAStartIdx + ((ea + 1) % numVertsA)];
    const dx = vertices[vaEdge * 3] - vertices[vbEdge * 3];
    const dy = vertices[vaEdge * 3 + 2] - vertices[vbEdge * 3 + 2];
    return { value: dx * dx + dy * dy, ea, eb };
};
const mergePolyVerts = (polygons, polyAStartIdx, polyBStartIdx, edgeIdxA, edgeIdxB, tempStartIdx, maxVerticesPerPoly) => {
    const numVertsA = countPolyVerts(polygons, polyAStartIdx, maxVerticesPerPoly);
    const numVertsB = countPolyVerts(polygons, polyBStartIdx, maxVerticesPerPoly);
    // clear tmp area
    for (let i = 0; i < maxVerticesPerPoly; i++) {
        polygons[tempStartIdx + i] = MESH_NULL_IDX;
    }
    let n = 0;
    // pdd pa
    for (let i = 0; i < numVertsA - 1; i++) {
        polygons[tempStartIdx + n++] = polygons[polyAStartIdx + ((edgeIdxA + 1 + i) % numVertsA)];
    }
    // add pb
    for (let i = 0; i < numVertsB - 1; i++) {
        polygons[tempStartIdx + n++] = polygons[polyBStartIdx + ((edgeIdxB + 1 + i) % numVertsB)];
    }
    // copy back to pa
    for (let i = 0; i < maxVerticesPerPoly; i++) {
        polygons[polyAStartIdx + i] = polygons[tempStartIdx + i];
    }
};
const buildPolyMesh = (ctx, contourSet, maxVerticesPerPoly) => {
    // calculate sizes
    let maxVertsPerCont = 0;
    for (let i = 0; i < contourSet.contours.length; i++) {
        const contour = contourSet.contours[i];
        if (contour.nVertices < 3)
            continue;
        maxVertsPerCont = Math.max(maxVertsPerCont, contour.nVertices);
    }
    // initialize mesh
    const mesh = {
        vertices: [],
        polys: [],
        regions: [],
        flags: [],
        areas: [],
        nVertices: 0,
        nPolys: 0,
        maxVerticesPerPoly,
        bounds: box3.clone(contourSet.bounds),
        localWidth: contourSet.width,
        localHeight: contourSet.height,
        cellSize: contourSet.cellSize,
        cellHeight: contourSet.cellHeight,
        borderSize: contourSet.borderSize,
        maxEdgeError: contourSet.maxError,
    };
    const vflags = [];
    const vertexMap = {};
    const indices = new Array(maxVertsPerCont);
    const tris = new Array(maxVertsPerCont * 3);
    const polys = new Array((maxVertsPerCont + 1) * maxVerticesPerPoly).fill(MESH_NULL_IDX);
    const tmpPolyStart = maxVertsPerCont * maxVerticesPerPoly;
    // process each contour
    for (let i = 0; i < contourSet.contours.length; i++) {
        const cont = contourSet.contours[i];
        if (cont.nVertices < 3)
            continue;
        // create indices
        for (let j = 0; j < cont.nVertices; j++) {
            indices[j] = j;
        }
        // triangulate contour
        let ntris = triangulate(cont.nVertices, cont.vertices, indices, tris);
        if (ntris <= 0) {
            BuildContext.warn(ctx, `Bad triangulation for contour ${i}`);
            ntris = Math.abs(ntris);
        }
        // add vertices
        for (let j = 0; j < cont.nVertices; j++) {
            const x = cont.vertices[j * 4];
            const y = cont.vertices[j * 4 + 1];
            const z = cont.vertices[j * 4 + 2];
            const flags = cont.vertices[j * 4 + 3];
            const idx = addVertex(x, y, z, mesh.vertices, vflags, vertexMap);
            indices[j] = idx;
            if (flags & BORDER_VERTEX) {
                vflags[idx] = 1;
            }
        }
        // build initial polygons
        let npolys = 0;
        polys.fill(MESH_NULL_IDX, 0, maxVertsPerCont * maxVerticesPerPoly);
        for (let j = 0; j < ntris; j++) {
            const t = [tris[j * 3], tris[j * 3 + 1], tris[j * 3 + 2]];
            if (t[0] !== t[1] && t[0] !== t[2] && t[1] !== t[2]) {
                polys[npolys * maxVerticesPerPoly] = indices[t[0]];
                polys[npolys * maxVerticesPerPoly + 1] = indices[t[1]];
                polys[npolys * maxVerticesPerPoly + 2] = indices[t[2]];
                npolys++;
            }
        }
        if (npolys === 0)
            continue;
        // merge polygons
        if (maxVerticesPerPoly > 3) {
            while (true) {
                let bestMergeVal = 0;
                let bestPa = 0;
                let bestPb = 0;
                let bestEa = 0;
                let bestEb = 0;
                for (let j = 0; j < npolys - 1; j++) {
                    for (let k = j + 1; k < npolys; k++) {
                        const paStart = j * maxVerticesPerPoly;
                        const pbStart = k * maxVerticesPerPoly;
                        const result = getPolyMergeValue(polys, paStart, pbStart, mesh.vertices, maxVerticesPerPoly);
                        if (result.value > bestMergeVal) {
                            bestMergeVal = result.value;
                            bestPa = j;
                            bestPb = k;
                            bestEa = result.ea;
                            bestEb = result.eb;
                        }
                    }
                }
                if (bestMergeVal > 0) {
                    const paStart = bestPa * maxVerticesPerPoly;
                    const pbStart = bestPb * maxVerticesPerPoly;
                    mergePolyVerts(polys, paStart, pbStart, bestEa, bestEb, tmpPolyStart, maxVerticesPerPoly);
                    // move last poly to fill gap
                    for (let m = 0; m < maxVerticesPerPoly; m++) {
                        polys[pbStart + m] = polys[(npolys - 1) * maxVerticesPerPoly + m];
                    }
                    npolys--;
                }
                else {
                    break;
                }
            }
        }
        // store polygons
        for (let j = 0; j < npolys; j++) {
            const pStart = mesh.nPolys * maxVerticesPerPoly;
            const qStart = j * maxVerticesPerPoly;
            for (let k = 0; k < maxVerticesPerPoly; k++) {
                mesh.polys[pStart + k] = polys[qStart + k];
            }
            mesh.regions[mesh.nPolys] = cont.reg;
            mesh.areas[mesh.nPolys] = cont.area;
            mesh.nPolys++;
        }
    }
    mesh.nVertices = Math.floor(mesh.vertices.length / 3);
    // remove edge vertices
    for (let i = 0; i < mesh.nVertices; i++) {
        if (vflags[i]) {
            if (!canRemoveVertex(mesh, i)) {
                continue;
            }
            if (removeVertex(ctx, mesh, i)) {
                // remove vertex - note: mesh.nVertices is already decremented inside removeVertex()!
                // fixup vertex flags
                for (let j = i; j < mesh.nVertices; j++) {
                    vflags[j] = vflags[j + 1];
                }
                i--;
            }
            else {
                BuildContext.error(ctx, `Failed to remove edge vertex ${i}`);
            }
        }
    }
    // trim arrays to size
    mesh.polys.length = mesh.nPolys * maxVerticesPerPoly;
    mesh.regions.length = mesh.nPolys;
    mesh.vertices.length = mesh.nVertices * 3;
    // allocate and initialize mesh flags array
    mesh.flags = new Array(mesh.nPolys).fill(0);
    return mesh;
};
const canRemoveVertex = (mesh, remVertexIdx) => {
    const nvp = mesh.maxVerticesPerPoly;
    // Count number of polygons to remove
    let numTouchedVerts = 0;
    let numRemainingEdges = 0;
    for (let i = 0; i < mesh.nPolys; i++) {
        const polyStart = i * nvp;
        const nv = countPolyVerts(mesh.polys, polyStart, nvp);
        let numRemoved = 0;
        let numVerts = 0;
        for (let j = 0; j < nv; j++) {
            if (mesh.polys[polyStart + j] === remVertexIdx) {
                numTouchedVerts++;
                numRemoved++;
            }
            numVerts++;
        }
        if (numRemoved) {
            numRemainingEdges += numVerts - (numRemoved + 1);
        }
    }
    // there would be too few edges remaining to create a polygon
    if (numRemainingEdges <= 2) {
        return false;
    }
    // find edges which share the removed vertex
    const maxEdges = numTouchedVerts * 2;
    const edges = new Array(maxEdges * 3);
    let nedges = 0;
    for (let i = 0; i < mesh.nPolys; i++) {
        const polyStart = i * nvp;
        const nv = countPolyVerts(mesh.polys, polyStart, nvp);
        // collect edges which touch the removed vertex
        for (let j = 0, k = nv - 1; j < nv; k = j++) {
            if (mesh.polys[polyStart + j] === remVertexIdx || mesh.polys[polyStart + k] === remVertexIdx) {
                // arrange edge so that a=rem
                let a = mesh.polys[polyStart + j];
                let b = mesh.polys[polyStart + k];
                if (b === remVertexIdx) {
                    // [a, b] = [b, a];
                    const tmp = a;
                    a = b;
                    b = tmp;
                }
                // Check if the edge exists
                let exists = false;
                for (let m = 0; m < nedges; m++) {
                    const e = m * 3;
                    if (edges[e + 1] === b) {
                        // Exists, increment vertex share count
                        edges[e + 2]++;
                        exists = true;
                        break;
                    }
                }
                // Add new edge
                if (!exists) {
                    const e = nedges * 3;
                    edges[e] = a;
                    edges[e + 1] = b;
                    edges[e + 2] = 1;
                    nedges++;
                }
            }
        }
    }
    // There should be no more than 2 open edges
    let numOpenEdges = 0;
    for (let i = 0; i < nedges; i++) {
        if (edges[i * 3 + 2] < 2) {
            numOpenEdges++;
        }
    }
    return numOpenEdges <= 2;
};
// Helper functions for hole building
const pushFront = (v, arr, an) => {
    an.value++;
    for (let i = an.value - 1; i > 0; i--) {
        arr[i] = arr[i - 1];
    }
    arr[0] = v;
};
const pushBack = (v, arr, an) => {
    arr[an.value] = v;
    an.value++;
};
const removeVertex = (ctx, mesh, remVertexIdx) => {
    const nvp = mesh.maxVerticesPerPoly;
    // Count number of polygons to remove
    let numRemovedVerts = 0;
    for (let i = 0; i < mesh.nPolys; i++) {
        const polyStart = i * nvp;
        const nv = countPolyVerts(mesh.polys, polyStart, nvp);
        for (let j = 0; j < nv; j++) {
            if (mesh.polys[polyStart + j] === remVertexIdx) {
                numRemovedVerts++;
            }
        }
    }
    const edges = new Array(numRemovedVerts * nvp * 4);
    let nedges = 0;
    const hole = new Array(numRemovedVerts * nvp);
    let nhole = 0;
    const hreg = new Array(numRemovedVerts * nvp);
    const harea = new Array(numRemovedVerts * nvp);
    for (let i = 0; i < mesh.nPolys; i++) {
        const polyStart = i * nvp;
        const nv = countPolyVerts(mesh.polys, polyStart, nvp);
        let hasRem = false;
        for (let j = 0; j < nv; j++) {
            if (mesh.polys[polyStart + j] === remVertexIdx) {
                hasRem = true;
                break;
            }
        }
        if (hasRem) {
            // Collect edges which do not touch the removed vertex
            for (let j = 0, k = nv - 1; j < nv; k = j++) {
                if (mesh.polys[polyStart + j] !== remVertexIdx && mesh.polys[polyStart + k] !== remVertexIdx) {
                    const e = nedges * 4;
                    edges[e] = mesh.polys[polyStart + k];
                    edges[e + 1] = mesh.polys[polyStart + j];
                    edges[e + 2] = mesh.regions[i];
                    edges[e + 3] = mesh.areas[i];
                    nedges++;
                }
            }
            // Remove the polygon
            const lastPolyStart = (mesh.nPolys - 1) * nvp;
            if (polyStart !== lastPolyStart) {
                for (let j = 0; j < nvp; j++) {
                    mesh.polys[polyStart + j] = mesh.polys[lastPolyStart + j];
                }
            }
            // Clear the last polygon
            for (let j = 0; j < nvp; j++) {
                mesh.polys[lastPolyStart + j] = MESH_NULL_IDX;
            }
            mesh.regions[i] = mesh.regions[mesh.nPolys - 1];
            mesh.areas[i] = mesh.areas[mesh.nPolys - 1];
            mesh.nPolys--;
            i--; // Reprocess this slot
        }
    }
    // Remove vertex
    for (let i = remVertexIdx; i < mesh.nVertices - 1; i++) {
        mesh.vertices[i * 3] = mesh.vertices[(i + 1) * 3];
        mesh.vertices[i * 3 + 1] = mesh.vertices[(i + 1) * 3 + 1];
        mesh.vertices[i * 3 + 2] = mesh.vertices[(i + 1) * 3 + 2];
    }
    mesh.nVertices--;
    // Adjust indices to match the removed vertex layout
    for (let i = 0; i < mesh.nPolys; i++) {
        const polyStart = i * nvp;
        const nv = countPolyVerts(mesh.polys, polyStart, nvp);
        for (let j = 0; j < nv; j++) {
            if (mesh.polys[polyStart + j] > remVertexIdx) {
                mesh.polys[polyStart + j]--;
            }
        }
    }
    for (let i = 0; i < nedges; i++) {
        if (edges[i * 4] > remVertexIdx)
            edges[i * 4]--;
        if (edges[i * 4 + 1] > remVertexIdx)
            edges[i * 4 + 1]--;
    }
    if (nedges === 0) {
        return true;
    }
    // Start with one vertex, keep appending connected segments
    const nholeRef = { value: 0 };
    const nhregRef = { value: 0 };
    const nhareaRef = { value: 0 };
    pushBack(edges[0], hole, nholeRef);
    pushBack(edges[2], hreg, nhregRef);
    pushBack(edges[3], harea, nhareaRef);
    nhole = nholeRef.value;
    while (nedges > 0) {
        let match = false;
        for (let i = 0; i < nedges; i++) {
            const ea = edges[i * 4];
            const eb = edges[i * 4 + 1];
            const r = edges[i * 4 + 2];
            const a = edges[i * 4 + 3];
            let add = false;
            if (hole[0] === eb) {
                // The segment matches the beginning of the hole boundary
                pushFront(ea, hole, nholeRef);
                pushFront(r, hreg, nhregRef);
                pushFront(a, harea, nhareaRef);
                add = true;
            }
            else if (hole[nhole - 1] === ea) {
                // The segment matches the end of the hole boundary
                pushBack(eb, hole, nholeRef);
                pushBack(r, hreg, nhregRef);
                pushBack(a, harea, nhareaRef);
                add = true;
            }
            if (add) {
                // The edge segment was added, remove it
                edges[i * 4] = edges[(nedges - 1) * 4];
                edges[i * 4 + 1] = edges[(nedges - 1) * 4 + 1];
                edges[i * 4 + 2] = edges[(nedges - 1) * 4 + 2];
                edges[i * 4 + 3] = edges[(nedges - 1) * 4 + 3];
                nedges--;
                match = true;
                i--;
            }
            nhole = nholeRef.value;
        }
        if (!match) {
            break;
        }
    }
    // generate temp vertex array for triangulation
    const tris = new Array(nhole * 3);
    const tverts = new Array(nhole * 4);
    const thole = new Array(nhole);
    for (let i = 0; i < nhole; i++) {
        const pi = hole[i];
        tverts[i * 4] = mesh.vertices[pi * 3];
        tverts[i * 4 + 1] = mesh.vertices[pi * 3 + 1];
        tverts[i * 4 + 2] = mesh.vertices[pi * 3 + 2];
        tverts[i * 4 + 3] = 0;
        thole[i] = i;
    }
    // triangulate the hole
    let ntris = triangulate(nhole, tverts, thole, tris);
    if (ntris < 0) {
        ntris = -ntris;
        BuildContext.warn(ctx, 'removeVertex: triangulate() returned bad results');
    }
    // merge the hole triangles back to polygons
    const polys = new Array((ntris + 1) * nvp);
    const pregs = new Array(ntris);
    const pareas = new Array(ntris);
    polys.fill(MESH_NULL_IDX);
    // build initial polygons
    let npolys = 0;
    for (let j = 0; j < ntris; j++) {
        const t = [tris[j * 3], tris[j * 3 + 1], tris[j * 3 + 2]];
        if (t[0] !== t[1] && t[0] !== t[2] && t[1] !== t[2]) {
            polys[npolys * nvp] = hole[t[0]];
            polys[npolys * nvp + 1] = hole[t[1]];
            polys[npolys * nvp + 2] = hole[t[2]];
            // If this polygon covers multiple region types then mark it as such
            if (hreg[t[0]] !== hreg[t[1]] || hreg[t[1]] !== hreg[t[2]]) {
                pregs[npolys] = MULTIPLE_REGS;
            }
            else {
                pregs[npolys] = hreg[t[0]];
            }
            pareas[npolys] = harea[t[0]];
            npolys++;
        }
    }
    if (npolys === 0) {
        return true;
    }
    // merge polygons
    if (nvp > 3) {
        while (true) {
            // find best polygons to merge
            let bestMergeVal = 0;
            let bestPa = 0;
            let bestPb = 0;
            let bestEa = 0;
            let bestEb = 0;
            for (let j = 0; j < npolys - 1; j++) {
                const pjStart = j * nvp;
                for (let k = j + 1; k < npolys; k++) {
                    const pkStart = k * nvp;
                    const result = getPolyMergeValue(polys, pjStart, pkStart, mesh.vertices, nvp);
                    if (result.value > bestMergeVal) {
                        bestMergeVal = result.value;
                        bestPa = j;
                        bestPb = k;
                        bestEa = result.ea;
                        bestEb = result.eb;
                    }
                }
            }
            if (bestMergeVal > 0) {
                // found best, merge
                const paStart = bestPa * nvp;
                const pbStart = bestPb * nvp;
                mergePolyVerts(polys, paStart, pbStart, bestEa, bestEb, ntris * nvp, nvp);
                if (pregs[bestPa] !== pregs[bestPb]) {
                    pregs[bestPa] = MULTIPLE_REGS;
                }
                const lastStart = (npolys - 1) * nvp;
                if (pbStart !== lastStart) {
                    for (let m = 0; m < nvp; m++) {
                        polys[pbStart + m] = polys[lastStart + m];
                    }
                }
                pregs[bestPb] = pregs[npolys - 1];
                pareas[bestPb] = pareas[npolys - 1];
                npolys--;
            }
            else {
                // could not merge any polygons, stop
                break;
            }
        }
    }
    // store polygons
    for (let i = 0; i < npolys; i++) {
        const meshPolyStart = mesh.nPolys * nvp;
        const polyStart = i * nvp;
        // clear the polygon
        for (let j = 0; j < nvp; j++) {
            mesh.polys[meshPolyStart + j] = MESH_NULL_IDX;
        }
        for (let j = 0; j < nvp; j++) {
            mesh.polys[meshPolyStart + j] = polys[polyStart + j];
        }
        mesh.regions[mesh.nPolys] = pregs[i];
        mesh.areas[mesh.nPolys] = pareas[i];
        mesh.nPolys++;
    }
    return true;
};

const DETAIL_EDGE_BOUNDARY = 0x1;
const UNSET_HEIGHT = 0xffff;
const MAX_VERTS = 127;
const MAX_TRIS = 255;
const MAX_VERTS_PER_EDGE = 32;
const RETRACT_SIZE = 256;
// Edge values enum
const EV_UNDEF = -1;
const EV_HULL = -2;
// Helper to extract 2D vector from 3D array (x, z components)
const getVec2XZ = (out, arr, index = 0) => {
    out[0] = arr[index]; // x component
    out[1] = arr[index + 2]; // z component (skip y)
    return out;
};
// Jitter functions for sampling
const getJitterX = (i) => {
    return (((i * 0x8da6b343) & 0xffff) / 65535.0) * 2.0 - 1.0;
};
const getJitterY = (i) => {
    return (((i * 0xd8163841) & 0xffff) / 65535.0) * 2.0 - 1.0;
};
// Helper functions for array navigation
const prev = (i, n) => (i - 1 >= 0 ? i - 1 : n - 1);
const next = (i, n) => (i + 1 < n ? i + 1 : 0);
// Height sampling function with spiral search
const getHeight = (fx, fy, fz, _cs, ics, ch, radius, hp) => {
    let ix = Math.floor(fx * ics + 0.01);
    let iz = Math.floor(fz * ics + 0.01);
    ix = clamp(ix - hp.xmin, 0, hp.width - 1);
    iz = clamp(iz - hp.ymin, 0, hp.height - 1);
    let h = hp.data[ix + iz * hp.width];
    if (h === UNSET_HEIGHT) {
        // Special case when data might be bad.
        // Walk adjacent cells in a spiral up to 'radius', and look
        // for a pixel which has a valid height.
        let x = 1;
        let z = 0;
        let dx = 1;
        let dz = 0;
        const maxSize = radius * 2 + 1;
        const maxIter = maxSize * maxSize - 1;
        let nextRingIterStart = 8;
        let nextRingIters = 16;
        let dmin = Number.MAX_VALUE;
        for (let i = 0; i < maxIter; i++) {
            const nx = ix + x;
            const nz = iz + z;
            if (nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height) {
                const nh = hp.data[nx + nz * hp.width];
                if (nh !== UNSET_HEIGHT) {
                    const d = Math.abs(nh * ch - fy);
                    if (d < dmin) {
                        h = nh;
                        dmin = d;
                    }
                }
            }
            // We are searching in a grid which looks approximately like this:
            //  __________
            // |2 ______ 2|
            // | |1 __ 1| |
            // | | |__| | |
            // | |______| |
            // |__________|
            // We want to find the best height as close to the center cell as possible. This means that
            // if we find a height in one of the neighbor cells to the center, we don't want to
            // expand further out than the 8 neighbors - we want to limit our search to the closest
            // of these "rings", but the best height in the ring.
            // For example, the center is just 1 cell. We checked that at the entrance to the function.
            // The next "ring" contains 8 cells (marked 1 above). Those are all the neighbors to the center cell.
            // The next one again contains 16 cells (marked 2). In general each ring has 8 additional cells, which
            // can be thought of as adding 2 cells around the "center" of each side when we expand the ring.
            // Here we detect if we are about to enter the next ring, and if we are and we have found
            // a height, we abort the search.
            if (i + 1 === nextRingIterStart) {
                if (h !== UNSET_HEIGHT)
                    break;
                nextRingIterStart += nextRingIters;
                nextRingIters += 8;
            }
            if (x === z || (x < 0 && x === -z) || (x > 0 && x === 1 - z)) {
                const tmp = dx;
                dx = -dz;
                dz = tmp;
            }
            x += dx;
            z += dz;
        }
    }
    return h;
};
// Edge management functions for triangulation
const findEdge = (edges, nedges, s, t) => {
    for (let i = 0; i < nedges; i++) {
        const e = i * 4;
        if ((edges[e] === s && edges[e + 1] === t) || (edges[e] === t && edges[e + 1] === s)) {
            return i;
        }
    }
    return EV_UNDEF;
};
const addEdge = (ctx, edges, nedges, maxEdges, s, t, l, r) => {
    if (nedges.value >= maxEdges) {
        BuildContext.error(ctx, `addEdge: Too many edges (${nedges.value}/${maxEdges}).`);
        return EV_UNDEF;
    }
    // Add edge if not already in the triangulation.
    const e = findEdge(edges, nedges.value, s, t);
    if (e === EV_UNDEF) {
        const edgeIdx = nedges.value * 4;
        edges[edgeIdx] = s;
        edges[edgeIdx + 1] = t;
        edges[edgeIdx + 2] = l;
        edges[edgeIdx + 3] = r;
        return nedges.value++;
    }
    return EV_UNDEF;
};
const _completeFacetC = vec3.create();
const _completeFacetPointS = vec2.create();
const _completeFacetPointT = vec2.create();
const _completeFacetPointU = vec2.create();
const _completeFacetCircleCenter = vec2.create();
const _completeFacetDistanceCalc = vec2.create();
const _circumcircleTriangle = triangle2.create();
const _circumcircleResult = circle.create();
// Triangle completion function for Delaunay triangulation
const completeFacet = (ctx, points, nPoints, edges, nEdges, maxEdges, nFaces, e) => {
    const EPS_FACET = 1e-5;
    const edgeIdx = e * 4;
    // Cache s and t.
    let s;
    let t;
    if (edges[edgeIdx + 2] === EV_UNDEF) {
        s = edges[edgeIdx];
        t = edges[edgeIdx + 1];
    }
    else if (edges[edgeIdx + 3] === EV_UNDEF) {
        s = edges[edgeIdx + 1];
        t = edges[edgeIdx];
    }
    else {
        // Edge already completed.
        return;
    }
    // Find best point on left of edge.
    let pt = nPoints;
    const c = vec3.set(_completeFacetC, 0, 0, 0);
    let r = -1;
    for (let u = 0; u < nPoints; ++u) {
        if (u === s || u === t)
            continue;
        // Calculate cross product to check if points are in correct order for triangle
        getVec2XZ(_completeFacetPointS, points, s * 3);
        getVec2XZ(_completeFacetPointT, points, t * 3);
        getVec2XZ(_completeFacetPointU, points, u * 3);
        vec2.subtract(_completeFacetPointT, _completeFacetPointT, _completeFacetPointS); // t - s
        vec2.subtract(_completeFacetPointU, _completeFacetPointU, _completeFacetPointS); // u - s
        const crossProduct = _completeFacetPointT[0] * _completeFacetPointU[1] - _completeFacetPointT[1] * _completeFacetPointU[0];
        if (crossProduct > EPS_FACET) {
            if (r < 0) {
                // The circle is not updated yet, do it now.
                pt = u;
                getVec2XZ(_circumcircleTriangle[0], points, s * 3);
                getVec2XZ(_circumcircleTriangle[1], points, t * 3);
                getVec2XZ(_circumcircleTriangle[2], points, u * 3);
                triangle2.circumcircle(_circumcircleResult, _circumcircleTriangle);
                c[0] = _circumcircleResult.center[0];
                c[1] = 0;
                c[2] = _circumcircleResult.center[1];
                r = _circumcircleResult.radius;
                continue;
            }
            getVec2XZ(_completeFacetCircleCenter, c, 0);
            getVec2XZ(_completeFacetDistanceCalc, points, u * 3);
            const d = vec2.distance(_completeFacetCircleCenter, _completeFacetDistanceCalc);
            const tol = 0.001;
            if (d > r * (1 + tol)) {
                // Outside current circumcircle, skip
                continue;
            }
            if (d >= r * (1 - tol)) {
                // Inside epsilon circumcircle, do extra tests to make sure the edge is valid.
                if (overlapEdges(points, edges, nEdges.value, s, u))
                    continue;
                if (overlapEdges(points, edges, nEdges.value, t, u))
                    continue;
            }
            // Edge is valid.
            pt = u;
            getVec2XZ(_circumcircleTriangle[0], points, s * 3);
            getVec2XZ(_circumcircleTriangle[1], points, t * 3);
            getVec2XZ(_circumcircleTriangle[2], points, u * 3);
            triangle2.circumcircle(_circumcircleResult, _circumcircleTriangle);
            c[0] = _circumcircleResult.center[0];
            c[1] = 0;
            c[2] = _circumcircleResult.center[1];
            r = _circumcircleResult.radius;
        }
    }
    // Add new triangle or update edge info if s-t is on hull.
    if (pt < nPoints) {
        // Update face information of edge being completed.
        updateLeftFace(edges, e, s, t, nFaces.value);
        // Add new edge or update face info of old edge.
        let newE = findEdge(edges, nEdges.value, pt, s);
        if (newE === EV_UNDEF) {
            addEdge(ctx, edges, nEdges, maxEdges, pt, s, nFaces.value, EV_UNDEF);
        }
        else {
            updateLeftFace(edges, newE, pt, s, nFaces.value);
        }
        // Add new edge or update face info of old edge.
        newE = findEdge(edges, nEdges.value, t, pt);
        if (newE === EV_UNDEF) {
            addEdge(ctx, edges, nEdges, maxEdges, t, pt, nFaces.value, EV_UNDEF);
        }
        else {
            updateLeftFace(edges, newE, t, pt, nFaces.value);
        }
        nFaces.value++;
    }
    else {
        updateLeftFace(edges, e, s, t, EV_HULL);
    }
};
const updateLeftFace = (edges, edgeIdx, s, t, f) => {
    const e = edgeIdx * 4;
    if (edges[e] === s && edges[e + 1] === t && edges[e + 2] === EV_UNDEF) {
        edges[e + 2] = f;
    }
    else if (edges[e + 1] === s && edges[e] === t && edges[e + 3] === EV_UNDEF) {
        edges[e + 3] = f;
    }
};
const _overlapEdgesS0 = vec2.create();
const _overlapEdgesT0 = vec2.create();
const _overlapEdgesS1 = vec2.create();
const _overlapEdgesT1 = vec2.create();
const overlapEdges = (pts, edges, nedges, s1, t1) => {
    for (let i = 0; i < nedges; ++i) {
        const s0 = edges[i * 4];
        const t0 = edges[i * 4 + 1];
        // Same or connected edges do not overlap.
        if (s0 === s1 || s0 === t1 || t0 === s1 || t0 === t1)
            continue;
        getVec2XZ(_overlapEdgesS0, pts, s0 * 3);
        getVec2XZ(_overlapEdgesT0, pts, t0 * 3);
        getVec2XZ(_overlapEdgesS1, pts, s1 * 3);
        getVec2XZ(_overlapEdgesT1, pts, t1 * 3);
        if (overlapSegSeg2d(_overlapEdgesS0, _overlapEdgesT0, _overlapEdgesS1, _overlapEdgesT1))
            return true;
    }
    return false;
};
// Delaunay triangulation hull function
const delaunayHull = (ctx, npts, pts, nhull, hull, tris, edges) => {
    const nfaces = { value: 0 };
    const nedges = { value: 0 };
    const maxEdges = npts * 10;
    // Resize edges array
    edges.length = maxEdges * 4;
    for (let i = 0, j = nhull - 1; i < nhull; j = i++) {
        addEdge(ctx, edges, nedges, maxEdges, hull[j], hull[i], EV_HULL, EV_UNDEF);
    }
    let currentEdge = 0;
    while (currentEdge < nedges.value) {
        if (edges[currentEdge * 4 + 2] === EV_UNDEF) {
            completeFacet(ctx, pts, npts, edges, nedges, maxEdges, nfaces, currentEdge);
        }
        if (edges[currentEdge * 4 + 3] === EV_UNDEF) {
            completeFacet(ctx, pts, npts, edges, nedges, maxEdges, nfaces, currentEdge);
        }
        currentEdge++;
    }
    // Create tris
    tris.length = nfaces.value * 4;
    for (let i = 0; i < nfaces.value * 4; ++i) {
        tris[i] = -1;
    }
    for (let i = 0; i < nedges.value; ++i) {
        const e = i * 4;
        if (edges[e + 3] >= 0) {
            // Left face
            const t = edges[e + 3] * 4;
            if (tris[t] === -1) {
                tris[t] = edges[e];
                tris[t + 1] = edges[e + 1];
            }
            else if (tris[t] === edges[e + 1]) {
                tris[t + 2] = edges[e];
            }
            else if (tris[t + 1] === edges[e]) {
                tris[t + 2] = edges[e + 1];
            }
        }
        if (edges[e + 2] >= 0) {
            // Right
            const t = edges[e + 2] * 4;
            if (tris[t] === -1) {
                tris[t] = edges[e + 1];
                tris[t + 1] = edges[e];
            }
            else if (tris[t] === edges[e]) {
                tris[t + 2] = edges[e + 1];
            }
            else if (tris[t + 1] === edges[e + 1]) {
                tris[t + 2] = edges[e];
            }
        }
    }
    // Remove dangling faces
    for (let i = 0; i < tris.length / 4; ++i) {
        const t = i * 4;
        if (tris[t] === -1 || tris[t + 1] === -1 || tris[t + 2] === -1) {
            BuildContext.warn(ctx, `delaunayHull: Removing dangling face ${i} [${tris[t]},${tris[t + 1]},${tris[t + 2]}].`);
            tris[t] = tris[tris.length - 4];
            tris[t + 1] = tris[tris.length - 3];
            tris[t + 2] = tris[tris.length - 2];
            tris[t + 3] = tris[tris.length - 1];
            tris.length -= 4;
            --i;
        }
    }
};
const triangulateHull = (verts, nhull, hull, nin, tris) => {
    let start = 0;
    let left = 1;
    let right = nhull - 1;
    // Start from an ear with shortest perimeter.
    let dmin = Number.MAX_VALUE;
    for (let i = 0; i < nhull; i++) {
        if (hull[i] >= nin)
            continue; // Ears are triangles with original vertices as middle vertex
        const pi = prev(i, nhull);
        const ni = next(i, nhull);
        const pv = hull[pi] * 3;
        const cv = hull[i] * 3;
        const nv = hull[ni] * 3;
        // Calculate triangle perimeter using 2D distances
        getVec2XZ(_triangulateHullPrev, verts, pv);
        getVec2XZ(_triangulateHullCurrent, verts, cv);
        getVec2XZ(_triangulateHullNext, verts, nv);
        const d = vec2.distance(_triangulateHullPrev, _triangulateHullCurrent) +
            vec2.distance(_triangulateHullCurrent, _triangulateHullNext) +
            vec2.distance(_triangulateHullNext, _triangulateHullPrev);
        if (d < dmin) {
            start = i;
            left = ni;
            right = pi;
            dmin = d;
        }
    }
    // Add first triangle
    tris.push(hull[start]);
    tris.push(hull[left]);
    tris.push(hull[right]);
    tris.push(0);
    // Triangulate the polygon by moving left or right
    while (next(left, nhull) !== right) {
        // Check to see if we should advance left or right.
        const nleft = next(left, nhull);
        const nright = prev(right, nhull);
        const cvleft = hull[left] * 3;
        const nvleft = hull[nleft] * 3;
        const cvright = hull[right] * 3;
        const nvright = hull[nright] * 3;
        // Calculate distances for left and right triangulation options
        getVec2XZ(_triangulateHullPrev, verts, cvleft);
        getVec2XZ(_triangulateHullCurrent, verts, nvleft);
        getVec2XZ(_triangulateHullNext, verts, cvright);
        getVec2XZ(_triangulateHullRight, verts, nvright);
        const dleft = vec2.distance(_triangulateHullPrev, _triangulateHullCurrent) +
            vec2.distance(_triangulateHullCurrent, _triangulateHullNext);
        const dright = vec2.distance(_triangulateHullNext, _triangulateHullRight) +
            vec2.distance(_triangulateHullPrev, _triangulateHullRight);
        if (dleft < dright) {
            tris.push(hull[left]);
            tris.push(hull[nleft]);
            tris.push(hull[right]);
            tris.push(0);
            left = nleft;
        }
        else {
            tris.push(hull[left]);
            tris.push(hull[nright]);
            tris.push(hull[right]);
            tris.push(0);
            right = nright;
        }
    }
};
// Check if edge is on hull
const onHull = (a, b, nhull, hull) => {
    // All internal sampled points come after the hull so we can early out for those.
    if (a >= nhull || b >= nhull)
        return false;
    for (let j = nhull - 1, i = 0; i < nhull; j = i++) {
        if (a === hull[j] && b === hull[i])
            return true;
    }
    return false;
};
// Set triangle flags for boundary edges
const setTriFlags = (tris, nhull, hull) => {
    for (let i = 0; i < tris.length; i += 4) {
        const a = tris[i];
        const b = tris[i + 1];
        const c = tris[i + 2];
        let flags = 0;
        flags |= (onHull(a, b, nhull, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 0;
        flags |= (onHull(b, c, nhull, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 2;
        flags |= (onHull(c, a, nhull, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 4;
        tris[i + 3] = flags;
    }
};
const SEED_ARRAY_WITH_POLY_CENTER_OFFSET = [
    [0, 0],
    [-1, -1],
    [0, -1],
    [1, -1],
    [1, 0],
    [1, 1],
    [0, 1],
    [-1, 1],
    [-1, 0],
];
// Seed array with polygon center for height data collection
const seedArrayWithPolyCenter = (ctx, chf, poly, polyStart, nPolys, verts, bs, hp, array) => {
    // Note: Reads to the compact heightfield are offset by border size (bs)
    // since border size offset is already removed from the polymesh vertices.
    const offset = SEED_ARRAY_WITH_POLY_CENTER_OFFSET;
    // Find cell closest to a poly vertex
    let startCellX = 0;
    let startCellY = 0;
    let startSpanIndex = -1;
    let dmin = UNSET_HEIGHT;
    for (let j = 0; j < nPolys && dmin > 0; ++j) {
        for (let k = 0; k < 9 && dmin > 0; ++k) {
            const ax = verts[poly[polyStart + j] * 3] + offset[k][0];
            const ay = verts[poly[polyStart + j] * 3 + 1];
            const az = verts[poly[polyStart + j] * 3 + 2] + offset[k][1];
            if (ax < hp.xmin || ax >= hp.xmin + hp.width || az < hp.ymin || az >= hp.ymin + hp.height)
                continue;
            const c = chf.cells[ax + bs + (az + bs) * chf.width];
            for (let i = c.index, ni = c.index + c.count; i < ni && dmin > 0; ++i) {
                const s = chf.spans[i];
                const d = Math.abs(ay - s.y);
                if (d < dmin) {
                    startCellX = ax;
                    startCellY = az;
                    startSpanIndex = i;
                    dmin = d;
                }
            }
        }
    }
    // Find center of the polygon
    let pcx = 0;
    let pcy = 0;
    for (let j = 0; j < nPolys; ++j) {
        pcx += verts[poly[polyStart + j] * 3];
        pcy += verts[poly[polyStart + j] * 3 + 2];
    }
    pcx = Math.floor(pcx / nPolys);
    pcy = Math.floor(pcy / nPolys);
    // Use seeds array as a stack for DFS
    array.length = 0;
    array.push(startCellX, startCellY, startSpanIndex);
    const dirs = [0, 1, 2, 3];
    hp.data.fill(0);
    // DFS to move to the center. Note that we need a DFS here and can not just move
    // directly towards the center without recording intermediate nodes, even though the polygons
    // are convex. In very rare we can get stuck due to contour simplification if we do not
    // record nodes.
    let cx = -1;
    let cy = -1;
    let ci = -1;
    while (true) {
        if (array.length < 3) {
            BuildContext.warn(ctx, 'Walk towards polygon center failed to reach center');
            break;
        }
        ci = array.pop();
        cy = array.pop();
        cx = array.pop();
        if (cx === pcx && cy === pcy)
            break;
        // If we are already at the correct X-position, prefer direction
        // directly towards the center in the Y-axis; otherwise prefer
        // direction in the X-axis
        let directDir;
        if (cx === pcx) {
            directDir = getDirForOffset(0, pcy > cy ? 1 : -1);
        }
        else {
            directDir = getDirForOffset(pcx > cx ? 1 : -1, 0);
        }
        // Push the direct dir last so we start with this on next iteration
        let temp = dirs[directDir];
        dirs[directDir] = dirs[3];
        dirs[3] = temp;
        const cs = chf.spans[ci];
        for (let i = 0; i < 4; i++) {
            const dir = dirs[i];
            if (getCon(cs, dir) === NOT_CONNECTED)
                continue;
            const newX = cx + getDirOffsetX(dir);
            const newY = cy + getDirOffsetY(dir);
            const hpx = newX - hp.xmin;
            const hpy = newY - hp.ymin;
            if (hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height)
                continue;
            if (hp.data[hpx + hpy * hp.width] !== 0)
                continue;
            hp.data[hpx + hpy * hp.width] = 1;
            array.push(newX, newY, chf.cells[newX + bs + (newY + bs) * chf.width].index + getCon(cs, dir));
        }
        // restore dirs array
        temp = dirs[directDir];
        dirs[directDir] = dirs[3];
        dirs[3] = temp;
    }
    array.length = 0;
    // getHeightData seeds are given in coordinates with borders
    array.push(cx + bs, cy + bs, ci);
    hp.data.fill(UNSET_HEIGHT);
    const cs = chf.spans[ci];
    hp.data[cx - hp.xmin + (cy - hp.ymin) * hp.width] = cs.y;
};
// Get height data for a polygon
const getHeightData = (ctx, chf, poly, polyStart, nPolys, verts, bs, hp, queue, region) => {
    // Note: Reads to the compact heightfield are offset by border size (bs)
    // since border size offset is already removed from the polymesh vertices.
    queue.length = 0;
    hp.data.fill(UNSET_HEIGHT);
    let empty = true;
    // We cannot sample from this poly if it was created from polys of different regions.
    if (region !== MULTIPLE_REGS) {
        // Copy the height from the same region, and mark region borders as seed points to fill the rest.
        for (let hy = 0; hy < hp.height; hy++) {
            const y = hp.ymin + hy + bs;
            for (let hx = 0; hx < hp.width; hx++) {
                const x = hp.xmin + hx + bs;
                const c = chf.cells[x + y * chf.width];
                for (let i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    const s = chf.spans[i];
                    if (s.region === region) {
                        // Store height
                        hp.data[hx + hy * hp.width] = s.y;
                        empty = false;
                        // If any of the neighbours is not in same region, add the current location as flood fill start
                        let border = false;
                        for (let dir = 0; dir < 4; ++dir) {
                            if (getCon(s, dir) !== NOT_CONNECTED) {
                                const ax = x + getDirOffsetX(dir);
                                const ay = y + getDirOffsetY(dir);
                                const ai = chf.cells[ax + ay * chf.width].index + getCon(s, dir);
                                const as = chf.spans[ai];
                                if (as.region !== region) {
                                    border = true;
                                    break;
                                }
                            }
                        }
                        if (border)
                            queue.push(x, y, i);
                        break;
                    }
                }
            }
        }
    }
    // if the polygon does not contain any points from the current region or if it could potentially be overlapping polygons
    if (empty) {
        seedArrayWithPolyCenter(ctx, chf, poly, polyStart, nPolys, verts, bs, hp, queue);
    }
    let head = 0;
    // BFS to collect height data
    while (head * 3 < queue.length) {
        const cx = queue[head * 3];
        const cy = queue[head * 3 + 1];
        const ci = queue[head * 3 + 2];
        head++;
        if (head >= RETRACT_SIZE) {
            head = 0;
            queue.splice(0, RETRACT_SIZE * 3);
        }
        const cs = chf.spans[ci];
        for (let dir = 0; dir < 4; ++dir) {
            if (getCon(cs, dir) === NOT_CONNECTED)
                continue;
            const ax = cx + getDirOffsetX(dir);
            const ay = cy + getDirOffsetY(dir);
            const hx = ax - hp.xmin - bs;
            const hy = ay - hp.ymin - bs;
            if (hx < 0 || hx >= hp.width || hy < 0 || hy >= hp.height)
                continue;
            if (hp.data[hx + hy * hp.width] !== UNSET_HEIGHT)
                continue;
            const ai = chf.cells[ax + ay * chf.width].index + getCon(cs, dir);
            const as = chf.spans[ai];
            hp.data[hx + hy * hp.width] = as.y;
            queue.push(ax, ay, ai);
        }
    }
};
const _buildPolyDetail_vj = vec3.create();
const _buildPolyDetail_vi = vec3.create();
const _buildPolyDetail_pt = vec3.create();
const _buildPolyDetail_va = vec3.create();
const _buildPolyDetail_vb = vec3.create();
const _buildPolyDetailSamplePt = vec3.create();
const _buildPolyDetailGridPt = vec3.create();
const _bmin$1 = vec3.create();
const _bmax$1 = vec3.create();
const buildPolyDetail = (ctx, inVerts, nin, sampleDist, sampleMaxError, heightSearchRadius, chf, hp, verts, tris, edges, samples) => {
    const edge = new Array((MAX_VERTS_PER_EDGE + 1) * 3);
    const hull = new Array(MAX_VERTS);
    let nhull = 0;
    let nverts = nin;
    // Copy input vertices
    for (let i = 0; i < nin; ++i) {
        verts[i * 3] = inVerts[i * 3];
        verts[i * 3 + 1] = inVerts[i * 3 + 1];
        verts[i * 3 + 2] = inVerts[i * 3 + 2];
    }
    // Clear arrays
    edges.length = 0;
    tris.length = 0;
    const cs = chf.cellSize;
    const ics = 1.0 / cs;
    // Calculate minimum extents of the polygon based on input data.
    const minExtent = polyMinExtent(verts, nverts);
    // Tessellate outlines.
    // This is done in separate pass in order to ensure
    // seamless height values across the ply boundaries.
    if (sampleDist > 0) {
        for (let i = 0, j = nin - 1; i < nin; j = i++) {
            let vjStart = j * 3;
            let viStart = i * 3;
            let swapped = false;
            // Make sure the segments are always handled in same order
            // using lexological sort or else there will be seams.
            if (Math.abs(inVerts[vjStart] - inVerts[viStart]) < 1e-6) {
                if (inVerts[vjStart + 2] > inVerts[viStart + 2]) {
                    const tmp = viStart;
                    viStart = vjStart;
                    vjStart = tmp;
                    swapped = true;
                }
            }
            else {
                if (inVerts[vjStart] > inVerts[viStart]) {
                    const tmp = viStart;
                    viStart = vjStart;
                    vjStart = tmp;
                    swapped = true;
                }
            }
            const vj = vec3.fromBuffer(_buildPolyDetail_vj, inVerts, vjStart);
            const vi = vec3.fromBuffer(_buildPolyDetail_vi, inVerts, viStart);
            // Create samples along the edge.
            const dx = vi[0] - vj[0];
            const dy = vi[1] - vj[1];
            const dz = vi[2] - vj[2];
            const d = Math.sqrt(dx * dx + dz * dz);
            let nn = 1 + Math.floor(d / sampleDist);
            if (nn >= MAX_VERTS_PER_EDGE)
                nn = MAX_VERTS_PER_EDGE - 1;
            if (nverts + nn >= MAX_VERTS)
                nn = MAX_VERTS - 1 - nverts;
            for (let k = 0; k <= nn; ++k) {
                const u = k / nn;
                const pos = k * 3;
                edge[pos] = vj[0] + dx * u;
                edge[pos + 1] = vj[1] + dy * u;
                edge[pos + 2] = vj[2] + dz * u;
                edge[pos + 1] =
                    getHeight(edge[pos], edge[pos + 1], edge[pos + 2], cs, ics, chf.cellHeight, heightSearchRadius, hp) *
                        chf.cellHeight;
            }
            // Simplify samples.
            const idx = new Array(MAX_VERTS_PER_EDGE).fill(0);
            idx[0] = 0;
            idx[1] = nn;
            let nidx = 2;
            for (let k = 0; k < nidx - 1;) {
                const a = idx[k];
                const b = idx[k + 1];
                const vaStart = a * 3;
                const vbStart = b * 3;
                // Find maximum deviation along the segment.
                let maxd = 0;
                let maxi = -1;
                for (let m = a + 1; m < b; ++m) {
                    const pt = vec3.fromBuffer(_buildPolyDetail_pt, edge, m * 3);
                    const va = vec3.fromBuffer(_buildPolyDetail_va, edge, vaStart);
                    const vb = vec3.fromBuffer(_buildPolyDetail_vb, edge, vbStart);
                    const dev = distancePtSeg$1(pt, va, vb);
                    if (dev > maxd) {
                        maxd = dev;
                        maxi = m;
                    }
                }
                // If the max deviation is larger than accepted error,
                // add new point, else continue to next segment.
                if (maxi !== -1 && maxd > sampleMaxError * sampleMaxError) {
                    for (let m = nidx; m > k; --m) {
                        idx[m] = idx[m - 1];
                    }
                    idx[k + 1] = maxi;
                    nidx++;
                }
                else {
                    ++k;
                }
            }
            hull[nhull++] = j;
            // Add new vertices.
            if (swapped) {
                for (let k = nidx - 2; k > 0; --k) {
                    verts[nverts * 3] = edge[idx[k] * 3];
                    verts[nverts * 3 + 1] = edge[idx[k] * 3 + 1];
                    verts[nverts * 3 + 2] = edge[idx[k] * 3 + 2];
                    hull[nhull++] = nverts;
                    nverts++;
                }
            }
            else {
                for (let k = 1; k < nidx - 1; ++k) {
                    verts[nverts * 3] = edge[idx[k] * 3];
                    verts[nverts * 3 + 1] = edge[idx[k] * 3 + 1];
                    verts[nverts * 3 + 2] = edge[idx[k] * 3 + 2];
                    hull[nhull++] = nverts;
                    nverts++;
                }
            }
        }
    }
    // If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
    if (minExtent < sampleDist * 2) {
        triangulateHull(verts, nhull, hull, nin, tris);
        setTriFlags(tris, nhull, hull);
        return true;
    }
    // Tessellate the base mesh using triangulateHull
    triangulateHull(verts, nhull, hull, nin, tris);
    if (tris.length === 0) {
        BuildContext.warn(ctx, `buildPolyDetail: Could not triangulate polygon (${nverts} verts).`);
        return true;
    }
    if (sampleDist > 0) {
        // Create sample locations in a grid.
        const bmin = vec3.set(_bmin$1, inVerts[0], inVerts[1], inVerts[2]);
        const bmax = vec3.set(_bmax$1, inVerts[0], inVerts[1], inVerts[2]);
        for (let i = 1; i < nin; ++i) {
            bmin[0] = Math.min(bmin[0], inVerts[i * 3]);
            bmin[1] = Math.min(bmin[1], inVerts[i * 3 + 1]);
            bmin[2] = Math.min(bmin[2], inVerts[i * 3 + 2]);
            bmax[0] = Math.max(bmax[0], inVerts[i * 3]);
            bmax[1] = Math.max(bmax[1], inVerts[i * 3 + 1]);
            bmax[2] = Math.max(bmax[2], inVerts[i * 3 + 2]);
        }
        const x0 = Math.floor(bmin[0] / sampleDist);
        const x1 = Math.ceil(bmax[0] / sampleDist);
        const z0 = Math.floor(bmin[2] / sampleDist);
        const z1 = Math.ceil(bmax[2] / sampleDist);
        samples.length = 0;
        for (let z = z0; z < z1; ++z) {
            for (let x = x0; x < x1; ++x) {
                const pt = [x * sampleDist, (bmax[1] + bmin[1]) * 0.5, z * sampleDist];
                // Make sure the samples are not too close to the edges.
                vec3.set(_buildPolyDetailGridPt, pt[0], pt[1], pt[2]);
                if (distToPoly(nin, inVerts, _buildPolyDetailGridPt) > -sampleDist / 2)
                    continue;
                samples.push(x);
                samples.push(getHeight(pt[0], pt[1], pt[2], cs, ics, chf.cellHeight, heightSearchRadius, hp));
                samples.push(z);
                samples.push(0); // Not added
            }
        }
        // Add the samples starting from the one that has the most error.
        const nsamples = samples.length / 4;
        for (let iter = 0; iter < nsamples; ++iter) {
            if (nverts >= MAX_VERTS)
                break;
            // Find sample with most error.
            const bestpt = [0, 0, 0];
            let bestd = 0;
            let besti = -1;
            for (let i = 0; i < nsamples; ++i) {
                const s = i * 4;
                if (samples[s + 3])
                    continue; // skip added.
                const pt = [
                    samples[s] * sampleDist + getJitterX(i) * cs * 0.1,
                    samples[s + 1] * chf.cellHeight,
                    samples[s + 2] * sampleDist + getJitterY(i) * cs * 0.1,
                ];
                vec3.set(_buildPolyDetailSamplePt, pt[0], pt[1], pt[2]);
                const d = distToTriMesh(_buildPolyDetailSamplePt, verts, tris, tris.length / 4);
                if (d < 0)
                    continue; // did not hit the mesh.
                if (d > bestd) {
                    bestd = d;
                    besti = i;
                    bestpt[0] = pt[0];
                    bestpt[1] = pt[1];
                    bestpt[2] = pt[2];
                }
            }
            // If the max error is within accepted threshold, stop tesselating.
            if (bestd <= sampleMaxError || besti === -1)
                break;
            // Mark sample as added.
            samples[besti * 4 + 3] = 1;
            // Add the new sample point.
            verts.push(bestpt[0], bestpt[1], bestpt[2]);
            nverts++;
            // Create new triangulation.
            // TODO: Incremental add instead of full rebuild.
            edges.length = 0;
            tris.length = 0;
            delaunayHull(ctx, nverts, verts, nhull, hull, tris, edges);
        }
    }
    const ntris = tris.length / 4;
    if (ntris > MAX_TRIS) {
        tris.length = MAX_TRIS * 4;
        BuildContext.warn(ctx, `buildPolyMeshDetail: Shrinking triangle count from ${ntris} to max ${MAX_TRIS}.`);
    }
    setTriFlags(tris, nhull, hull);
    return true;
};
const _triangulateHullPrev = vec2.create();
const _triangulateHullCurrent = vec2.create();
const _triangulateHullNext = vec2.create();
const _triangulateHullRight = vec2.create();
const buildPolyMeshDetail = (ctx, polyMesh, compactHeightfield, sampleDist, sampleMaxError) => {
    if (polyMesh.nVertices === 0 || polyMesh.nPolys === 0) {
        return {
            meshes: [],
            vertices: [],
            triangles: [],
            nMeshes: 0,
            nVertices: 0,
            nTriangles: 0,
        };
    }
    const nvp = polyMesh.maxVerticesPerPoly;
    const cs = polyMesh.cellSize;
    const ch = polyMesh.cellHeight;
    const orig = [polyMesh.bounds[0][0], polyMesh.bounds[0][1], polyMesh.bounds[0][2]];
    const borderSize = polyMesh.borderSize;
    const heightSearchRadius = Math.max(1, Math.ceil(polyMesh.maxEdgeError));
    const hp = {
        data: [],
        xmin: 0,
        ymin: 0,
        width: 0,
        height: 0,
    };
    let maxhw = 0;
    let maxhh = 0;
    // Calculate bounds for each polygon
    const bounds = new Array(polyMesh.nPolys * 4);
    const poly = new Array(nvp * 3);
    // Find max size for a polygon area.
    for (let i = 0; i < polyMesh.nPolys; ++i) {
        const p = i * nvp;
        let xmin = compactHeightfield.width;
        let xmax = 0;
        let ymin = compactHeightfield.height;
        let ymax = 0;
        for (let j = 0; j < nvp; ++j) {
            if (polyMesh.polys[p + j] === MESH_NULL_IDX)
                break;
            const v = polyMesh.polys[p + j] * 3;
            xmin = Math.min(xmin, polyMesh.vertices[v]);
            xmax = Math.max(xmax, polyMesh.vertices[v]);
            ymin = Math.min(ymin, polyMesh.vertices[v + 2]);
            ymax = Math.max(ymax, polyMesh.vertices[v + 2]);
        }
        bounds[i * 4] = Math.max(0, xmin - 1);
        bounds[i * 4 + 1] = Math.min(compactHeightfield.width, xmax + 1);
        bounds[i * 4 + 2] = Math.max(0, ymin - 1);
        bounds[i * 4 + 3] = Math.min(compactHeightfield.height, ymax + 1);
        if (bounds[i * 4] >= bounds[i * 4 + 1] || bounds[i * 4 + 2] >= bounds[i * 4 + 3])
            continue;
        maxhw = Math.max(maxhw, bounds[i * 4 + 1] - bounds[i * 4]);
        maxhh = Math.max(maxhh, bounds[i * 4 + 3] - bounds[i * 4 + 2]);
    }
    hp.data = new Array(maxhw * maxhh);
    const dmesh = {
        meshes: new Array(polyMesh.nPolys * 4),
        vertices: [],
        triangles: [],
        nMeshes: polyMesh.nPolys,
        nVertices: 0,
        nTriangles: 0,
    };
    for (let i = 0; i < polyMesh.nPolys; ++i) {
        const p = i * nvp;
        const edges = [];
        const tris = [];
        const arr = [];
        const samples = [];
        const verts = [];
        // Store polygon vertices for processing.
        let npoly = 0;
        for (let j = 0; j < nvp; ++j) {
            if (polyMesh.polys[p + j] === MESH_NULL_IDX)
                break;
            const v = polyMesh.polys[p + j] * 3;
            poly[j * 3] = polyMesh.vertices[v] * cs;
            poly[j * 3 + 1] = polyMesh.vertices[v + 1] * ch;
            poly[j * 3 + 2] = polyMesh.vertices[v + 2] * cs;
            npoly++;
        }
        // Get the height data from the area of the polygon.
        hp.xmin = bounds[i * 4];
        hp.ymin = bounds[i * 4 + 2];
        hp.width = bounds[i * 4 + 1] - bounds[i * 4];
        hp.height = bounds[i * 4 + 3] - bounds[i * 4 + 2];
        getHeightData(ctx, compactHeightfield, polyMesh.polys, p, npoly, polyMesh.vertices, borderSize, hp, arr, polyMesh.regions[i]);
        // Build detail mesh.
        if (!buildPolyDetail(ctx, poly, npoly, sampleDist, sampleMaxError, heightSearchRadius, compactHeightfield, hp, verts, tris, edges, samples)) {
            BuildContext.error(ctx, `buildPolyMeshDetail: Failed to build detail mesh for poly ${i}.`);
            continue;
        }
        // Move detail verts to world space.
        for (let i = 0; i < verts.length; i += 3) {
            verts[i] += orig[0];
            verts[i + 1] += orig[1] + compactHeightfield.cellHeight; // Is this offset necessary?
            verts[i + 2] += orig[2];
        }
        // Offset poly too, will be used to flag checking.
        for (let j = 0; j < npoly; ++j) {
            poly[j * 3] += orig[0];
            poly[j * 3 + 1] += orig[1];
            poly[j * 3 + 2] += orig[2];
        }
        // Store detail submesh.
        const ntris = tris.length / 4;
        dmesh.meshes[i * 4] = dmesh.nVertices;
        dmesh.meshes[i * 4 + 1] = verts.length / 3;
        dmesh.meshes[i * 4 + 2] = dmesh.nTriangles;
        dmesh.meshes[i * 4 + 3] = ntris;
        // Store vertices
        for (let i = 0; i < verts.length; i += 3) {
            dmesh.vertices.push(verts[i], verts[i + 1], verts[i + 2]);
            dmesh.nVertices++;
        }
        // Store triangles
        for (let i = 0; i < tris.length; i += 4) {
            dmesh.triangles.push(tris[i], tris[i + 1], tris[i + 2], tris[i + 3]);
            dmesh.nTriangles++;
        }
    }
    return dmesh;
};

const createIndexPool = () => {
    return {
        free: [],
        counter: 0,
    };
};
const requestIndex = (indexPool) => {
    if (indexPool.free.length > 0) {
        return indexPool.free.pop();
    }
    return indexPool.counter++;
};
const releaseIndex = (indexPool, index) => {
    indexPool.free.push(index);
};

const compareItemX = (a, b) => {
    if (a.bounds[0][0] < b.bounds[0][0])
        return -1;
    if (a.bounds[0][0] > b.bounds[0][0])
        return 1;
    return 0;
};
const compareItemY = (a, b) => {
    if (a.bounds[0][1] < b.bounds[0][1])
        return -1;
    if (a.bounds[0][1] > b.bounds[0][1])
        return 1;
    return 0;
};
const compareItemZ = (a, b) => {
    if (a.bounds[0][2] < b.bounds[0][2])
        return -1;
    if (a.bounds[0][2] > b.bounds[0][2])
        return 1;
    return 0;
};
const calcExtends = (items, imin, imax) => {
    const bounds = [
        [items[imin].bounds[0][0], items[imin].bounds[0][1], items[imin].bounds[0][2]],
        [items[imin].bounds[1][0], items[imin].bounds[1][1], items[imin].bounds[1][2]],
    ];
    for (let i = imin + 1; i < imax; ++i) {
        const it = items[i];
        if (it.bounds[0][0] < bounds[0][0])
            bounds[0][0] = it.bounds[0][0];
        if (it.bounds[0][1] < bounds[0][1])
            bounds[0][1] = it.bounds[0][1];
        if (it.bounds[0][2] < bounds[0][2])
            bounds[0][2] = it.bounds[0][2];
        if (it.bounds[1][0] > bounds[1][0])
            bounds[1][0] = it.bounds[1][0];
        if (it.bounds[1][1] > bounds[1][1])
            bounds[1][1] = it.bounds[1][1];
        if (it.bounds[1][2] > bounds[1][2])
            bounds[1][2] = it.bounds[1][2];
    }
    return bounds;
};
const longestAxis = (x, y, z) => {
    let axis = 0;
    let maxVal = x;
    if (y > maxVal) {
        axis = 1;
        maxVal = y;
    }
    if (z > maxVal) {
        axis = 2;
    }
    return axis;
};
const subdivide = (items, imin, imax, curNode, nodes) => {
    const inum = imax - imin;
    const icur = curNode.value;
    const node = {
        bounds: [
            [0, 0, 0],
            [0, 0, 0],
        ],
        i: 0,
    };
    nodes[curNode.value++] = node;
    if (inum === 1) {
        // Leaf
        node.bounds[0][0] = items[imin].bounds[0][0];
        node.bounds[0][1] = items[imin].bounds[0][1];
        node.bounds[0][2] = items[imin].bounds[0][2];
        node.bounds[1][0] = items[imin].bounds[1][0];
        node.bounds[1][1] = items[imin].bounds[1][1];
        node.bounds[1][2] = items[imin].bounds[1][2];
        node.i = items[imin].i;
    }
    else {
        // Split
        const extents = calcExtends(items, imin, imax);
        node.bounds[0][0] = extents[0][0];
        node.bounds[0][1] = extents[0][1];
        node.bounds[0][2] = extents[0][2];
        node.bounds[1][0] = extents[1][0];
        node.bounds[1][1] = extents[1][1];
        node.bounds[1][2] = extents[1][2];
        const axis = longestAxis(node.bounds[1][0] - node.bounds[0][0], node.bounds[1][1] - node.bounds[0][1], node.bounds[1][2] - node.bounds[0][2]);
        if (axis === 0) {
            // Sort along x-axis
            const segment = items.slice(imin, imax);
            segment.sort(compareItemX);
            for (let i = 0; i < segment.length; i++) {
                items[imin + i] = segment[i];
            }
        }
        else if (axis === 1) {
            // Sort along y-axis
            const segment = items.slice(imin, imax);
            segment.sort(compareItemY);
            for (let i = 0; i < segment.length; i++) {
                items[imin + i] = segment[i];
            }
        }
        else {
            // Sort along z-axis
            const segment = items.slice(imin, imax);
            segment.sort(compareItemZ);
            for (let i = 0; i < segment.length; i++) {
                items[imin + i] = segment[i];
            }
        }
        const isplit = imin + Math.floor(inum / 2);
        // Left
        subdivide(items, imin, isplit, curNode, nodes);
        // Right
        subdivide(items, isplit, imax, curNode, nodes);
        const iescape = curNode.value - icur;
        // Negative index means escape.
        node.i = -iescape;
    }
};
/**
 * Builds a bounding volume tree for the given nav mesh tile.
 * @param navMeshTile the nav mesh tile to build the BV tree for
 * @returns
 */
const buildNavMeshBvTree = (params) => {
    // use cellSize for quantization factor
    const quantFactor = 1 / params.cellSize;
    // early exit if the tile has no polys
    if (params.polys.length === 0) {
        return {
            nodes: [],
            quantFactor,
        };
    }
    // allocate bv tree nodes for polys
    const items = new Array(Object.keys(params.polys).length);
    // calculate bounds for each polygon
    for (let i = 0; i < params.polys.length; i++) {
        const item = {
            bounds: [
                [0, 0, 0],
                [0, 0, 0],
            ],
            i,
        };
        const poly = params.polys[i];
        const nvp = poly.vertices.length;
        if (nvp > 0) {
            // expand bounds with polygon vertices
            const firstVertIndex = poly.vertices[0] * 3;
            item.bounds[0][0] = item.bounds[1][0] = params.vertices[firstVertIndex];
            item.bounds[0][1] = item.bounds[1][1] = params.vertices[firstVertIndex + 1];
            item.bounds[0][2] = item.bounds[1][2] = params.vertices[firstVertIndex + 2];
            for (let j = 1; j < nvp; j++) {
                const vertexIndex = poly.vertices[j];
                if (vertexIndex === MESH_NULL_IDX)
                    break;
                const vertIndex = vertexIndex * 3;
                const x = params.vertices[vertIndex];
                const y = params.vertices[vertIndex + 1];
                const z = params.vertices[vertIndex + 2];
                if (x < item.bounds[0][0])
                    item.bounds[0][0] = x;
                if (y < item.bounds[0][1])
                    item.bounds[0][1] = y;
                if (z < item.bounds[0][2])
                    item.bounds[0][2] = z;
                if (x > item.bounds[1][0])
                    item.bounds[1][0] = x;
                if (y > item.bounds[1][1])
                    item.bounds[1][1] = y;
                if (z > item.bounds[1][2])
                    item.bounds[1][2] = z;
            }
            // expand bounds with additional detail vertices if available
            if (params.detailMeshes.length > 0 && params.detailVertices.length > 0) {
                const detailMesh = params.detailMeshes[i];
                const vb = detailMesh.verticesBase;
                const ndv = detailMesh.verticesCount;
                // iterate through additional detail vertices (not including poly vertices)
                for (let j = 0; j < ndv; j++) {
                    const vertIndex = (vb + j) * 3;
                    const x = params.detailVertices[vertIndex];
                    const y = params.detailVertices[vertIndex + 1];
                    const z = params.detailVertices[vertIndex + 2];
                    if (x < item.bounds[0][0])
                        item.bounds[0][0] = x;
                    if (y < item.bounds[0][1])
                        item.bounds[0][1] = y;
                    if (z < item.bounds[0][2])
                        item.bounds[0][2] = z;
                    if (x > item.bounds[1][0])
                        item.bounds[1][0] = x;
                    if (y > item.bounds[1][1])
                        item.bounds[1][1] = y;
                    if (z > item.bounds[1][2])
                        item.bounds[1][2] = z;
                }
            }
            // bv tree uses cellSize for all dimensions, quantize relative to tile bounds
            item.bounds[0][0] = (item.bounds[0][0] - params.bounds[0][0]) * quantFactor;
            item.bounds[0][1] = (item.bounds[0][1] - params.bounds[0][1]) * quantFactor;
            item.bounds[0][2] = (item.bounds[0][2] - params.bounds[0][2]) * quantFactor;
            item.bounds[1][0] = (item.bounds[1][0] - params.bounds[0][0]) * quantFactor;
            item.bounds[1][1] = (item.bounds[1][1] - params.bounds[0][1]) * quantFactor;
            item.bounds[1][2] = (item.bounds[1][2] - params.bounds[0][2]) * quantFactor;
        }
        items[i] = item;
    }
    const curNode = { value: 0 };
    const nPolys = Object.keys(params.polys).length;
    const nodes = new Array(nPolys * 2);
    subdivide(items, 0, nPolys, curNode, nodes);
    // trim the nodes array to actual size
    const trimmedNodes = nodes.slice(0, curNode.value);
    const bvTree = {
        nodes: trimmedNodes,
        quantFactor: quantFactor,
    };
    return bvTree;
};

var OffMeshConnectionDirection;
(function (OffMeshConnectionDirection) {
    OffMeshConnectionDirection[OffMeshConnectionDirection["START_TO_END"] = 0] = "START_TO_END";
    OffMeshConnectionDirection[OffMeshConnectionDirection["BIDIRECTIONAL"] = 1] = "BIDIRECTIONAL";
})(OffMeshConnectionDirection || (OffMeshConnectionDirection = {}));

var NodeType;
(function (NodeType) {
    /** the node is a standard ground convex polygon that is part of the surface of the mesh */
    NodeType[NodeType["POLY"] = 0] = "POLY";
    /** the node is an off-mesh connection */
    NodeType[NodeType["OFFMESH"] = 1] = "OFFMESH";
})(NodeType || (NodeType = {}));
/** Invalid node reference constant */
const INVALID_NODE_REF = -1;
const TYPE_BITS = 1;
const NODE_INDEX_BITS = 31;
const SEQUENCE_BITS = 20;
// masks for 32-bit operations (bits 1-32)
const TYPE_MASK = 0x1; // bit 1
const NODE_INDEX_MASK = 0x7FFFFFFF; // bits 2-32 (31 bits)
const NODE_INDEX_SHIFT = TYPE_BITS; // 1
// sequence number uses bits beyond 32-bit boundary (bits 33-52)
const SEQUENCE_SHIFT = TYPE_BITS + NODE_INDEX_BITS; // 32
const SEQUENCE_MASK = (1 << SEQUENCE_BITS) - 1; // 0xFFFFF (20 bits)
// maximum values for each field based on bit allocation
const MAX_NODE_INDEX = NODE_INDEX_MASK; // 2147483647 (31 bits: 2^31 - 1)
const MAX_SEQUENCE = SEQUENCE_MASK; // 1048575 (20 bits: 2^20 - 1)
/** Serializes a node reference from its components */
const serNodeRef = (type, nodeIndex, sequence) => {
    // NOTE: mask inputs to avoid accidental overflow
    const t = type & TYPE_MASK;
    const n = nodeIndex & NODE_INDEX_MASK;
    const s = sequence & SEQUENCE_MASK;
    // Pack as: [type: 1 bit][nodeIndex: 31 bits][sequence: 20 bits]
    // Use multiplication instead of bitwise shift to avoid 32-bit truncation, we encode sequence in higher bits
    return t + (n * 2) + (s * 2 ** SEQUENCE_SHIFT);
};
/** Gets the node type from a node reference */
const getNodeRefType = (ref) => {
    // fast truncated 32-bit operation
    return (ref & TYPE_MASK);
};
/** Gets the node index from a node reference */
const getNodeRefIndex = (ref) => {
    // fast truncated 32-bit operation
    return (ref >>> NODE_INDEX_SHIFT) & NODE_INDEX_MASK;
};
/** Gets the sequence number from a node reference */
const getNodeRefSequence = (ref) => {
    // non-32-bit operation, use division for bits beyond 32-bit boundary
    return Math.floor(ref / 2 ** SEQUENCE_SHIFT) & SEQUENCE_MASK;
};

/**
 * Creates a new empty navigation mesh.
 * @returns The created navigation mesh
 */
const createNavMesh = () => {
    return {
        origin: [0, 0, 0],
        tileWidth: 0,
        tileHeight: 0,
        links: [],
        nodes: [],
        tiles: {},
        tilePositionToTileId: {},
        tileColumnToTileIds: {},
        offMeshConnections: {},
        offMeshConnectionAttachments: {},
        tilePositionToSequenceCounter: {},
        offMeshConnectionSequenceCounter: 0,
        nodeIndexPool: createIndexPool(),
        tileIndexPool: createIndexPool(),
        offMeshConnectionIndexPool: createIndexPool(),
        linkIndexPool: createIndexPool(),
    };
};
/**
 * Gets a navigation mesh node by its reference.
 * Note that navmesh nodes are pooled and may be reused on removing then adding tiles, so do not store node objects.
 * @param navMesh the navigation mesh
 * @param nodeRef the node reference
 * @returns the navigation mesh node
 */
const getNodeByRef = (navMesh, nodeRef) => {
    const nodeIndex = getNodeRefIndex(nodeRef);
    const node = navMesh.nodes[nodeIndex];
    return node;
};
/**
 * Gets a navigation mesh node by its tile and polygon index.
 * @param navMesh the navigation mesh
 * @param tile the navigation mesh tile
 * @param polyIndex the polygon index
 * @returns the navigation mesh node
 */
const getNodeByTileAndPoly = (navMesh, tile, polyIndex) => {
    const navMeshNodeIndex = tile.polyNodes[polyIndex];
    const navMeshNode = navMesh.nodes[navMeshNodeIndex];
    return navMeshNode;
};
/**
 * Checks if a navigation mesh node reference is valid.
 * @param navMesh the navigation mesh
 * @param nodeRef the node reference
 * @returns true if the node reference is valid, false otherwise
 */
const isValidNodeRef = (navMesh, nodeRef) => {
    if (nodeRef === INVALID_NODE_REF) {
        return false;
    }
    const nodeType = getNodeRefType(nodeRef);
    if (nodeType === NodeType.POLY) {
        const node = getNodeByRef(navMesh, nodeRef);
        if (!node) {
            return false;
        }
        const tile = navMesh.tiles[node.tileId];
        if (!tile) {
            return false;
        }
        const sequence = getNodeRefSequence(nodeRef);
        if (tile.sequence !== sequence) {
            return false;
        }
        if (node.polyIndex < 0 || node.polyIndex >= tile.polys.length) {
            return false;
        }
        const poly = tile.polys[node.polyIndex];
        if (!poly) {
            return false;
        }
        return true;
    }
    if (nodeType === NodeType.OFFMESH) {
        const node = getNodeByRef(navMesh, nodeRef);
        if (!node) {
            return false;
        }
        const offMeshConnection = navMesh.offMeshConnections[node.offMeshConnectionId];
        if (!offMeshConnection) {
            return false;
        }
        const sequence = getNodeRefSequence(nodeRef);
        if (offMeshConnection.sequence !== sequence) {
            return false;
        }
        if (!isOffMeshConnectionConnected(navMesh, offMeshConnection.id)) {
            return false;
        }
        return true;
    }
    return false;
};
/**
 * Gets the tile at the given x, y, and layer position.
 * @param navMesh the navigation mesh
 * @param x the x position
 * @param y the y position
 * @param layer the layer
 * @returns the navigation mesh tile
 */
const getTileAt = (navMesh, x, y, layer) => {
    const tileHash = getTilePositionHash(x, y, layer);
    const tileId = navMesh.tilePositionToTileId[tileHash];
    return navMesh.tiles[tileId];
};
/**
 * Gets all tiles at the given x and y position.
 * @param navMesh the navigation mesh
 * @param x the x position
 * @param y the y position
 * @returns the navigation mesh tiles
 */
const getTilesAt = (navMesh, x, y) => {
    const tileColumnHash = getTileColumnHash(x, y);
    const tileIds = navMesh.tileColumnToTileIds[tileColumnHash];
    if (!tileIds)
        return [];
    const tiles = [];
    for (const tileId of tileIds) {
        tiles.push(navMesh.tiles[tileId]);
    }
    return tiles;
};
const getNeighbourTilesAt = (navMesh, x, y, side) => {
    let nx = x;
    let ny = y;
    switch (side) {
        case 0:
            nx++;
            break;
        case 1:
            nx++;
            ny++;
            break;
        case 2:
            ny++;
            break;
        case 3:
            nx--;
            ny++;
            break;
        case 4:
            nx--;
            break;
        case 5:
            nx--;
            ny--;
            break;
        case 6:
            ny--;
            break;
        case 7:
            nx++;
            ny--;
            break;
    }
    return getTilesAt(navMesh, nx, ny);
};
const getTilePositionHash = (x, y, layer) => {
    return `${x},${y},${layer}`;
};
const getTileColumnHash = (x, y) => {
    return `${x},${y}`;
};
/**
 * Returns the tile x and y position in the nav mesh from a world space position.
 * @param outTilePosition the output tile position
 * @param navMesh the navigation mesh
 * @param worldPosition the world space position
 */
const worldToTilePosition = (outTilePosition, navMesh, worldPosition) => {
    outTilePosition[0] = Math.floor((worldPosition[0] - navMesh.origin[0]) / navMesh.tileWidth);
    outTilePosition[1] = Math.floor((worldPosition[2] - navMesh.origin[2]) / navMesh.tileHeight);
    return outTilePosition;
};
/**
 * Gets the tile and polygon from a polygon reference
 * @param ref The polygon reference
 * @param navMesh The navigation mesh
 * @returns Object containing tile and poly, or null if not found
 */
const getTileAndPolyByRef = (ref, navMesh) => {
    const result = {
        success: false,
        tile: null,
        poly: null,
        polyIndex: -1,
    };
    const nodeType = getNodeRefType(ref);
    if (nodeType !== NodeType.POLY)
        return result;
    const { tileId, polyIndex } = getNodeByRef(navMesh, ref);
    const tile = navMesh.tiles[tileId];
    if (!tile) {
        return result;
    }
    if (polyIndex >= tile.polys.length) {
        return result;
    }
    result.poly = tile.polys[polyIndex];
    result.tile = tile;
    result.polyIndex = polyIndex;
    result.success = true;
    return result;
};
const _getPolyHeightA = vec3.create();
const _getPolyHeightB = vec3.create();
const _getPolyHeightC = vec3.create();
const _getPolyHeightTriangle = [_getPolyHeightA, _getPolyHeightB, _getPolyHeightC];
const _getPolyHeightVertices = [];
const createGetPolyHeightResult = () => ({
    success: false,
    height: 0,
});
/**
 * Gets the height of a polygon at a given point using detail mesh if available.
 * @param result The result object to populate
 * @param tile The tile containing the polygon
 * @param poly The polygon
 * @param polyIndex The index of the polygon in the tile
 * @param pos The position to get height for
 * @returns The result object with success flag and height
 */
const getPolyHeight = (result, tile, poly, polyIndex, pos) => {
    result.success = false;
    result.height = 0;
    const detailMesh = tile.detailMeshes[polyIndex];
    // build polygon vertices array
    const nv = poly.vertices.length;
    const vertices = _getPolyHeightVertices;
    for (let i = 0; i < nv; ++i) {
        const start = poly.vertices[i] * 3;
        vertices[i * 3] = tile.vertices[start];
        vertices[i * 3 + 1] = tile.vertices[start + 1];
        vertices[i * 3 + 2] = tile.vertices[start + 2];
    }
    // check if point is inside polygon
    if (!pointInPoly(pos, vertices, nv)) {
        return result;
    }
    // point is inside polygon, find height at the location
    if (detailMesh) {
        for (let j = 0; j < detailMesh.trianglesCount; ++j) {
            const t = (detailMesh.trianglesBase + j) * 4;
            const detailTriangles = tile.detailTriangles;
            // get triangle vertices
            const v = _getPolyHeightTriangle;
            for (let k = 0; k < 3; ++k) {
                const vertIndex = detailTriangles[t + k];
                if (vertIndex < poly.vertices.length) {
                    // use polygon vertex
                    const polyVertIndex = poly.vertices[vertIndex] * 3;
                    v[k][0] = tile.vertices[polyVertIndex + 0];
                    v[k][1] = tile.vertices[polyVertIndex + 1];
                    v[k][2] = tile.vertices[polyVertIndex + 2];
                }
                else {
                    // use detail vertices
                    const detailVertIndex = (detailMesh.verticesBase + (vertIndex - poly.vertices.length)) * 3;
                    v[k][0] = tile.detailVertices[detailVertIndex + 0];
                    v[k][1] = tile.detailVertices[detailVertIndex + 1];
                    v[k][2] = tile.detailVertices[detailVertIndex + 2];
                }
            }
            const height = closestHeightPointTriangle(pos, v[0], v[1], v[2]);
            if (!Number.isNaN(height)) {
                result.success = true;
                result.height = height;
                return result;
            }
        }
    }
    // if all triangle checks failed above (can happen with degenerate triangles
    // or larger floating point values) the point is on an edge, so just select
    // closest.
    // this should almost never happen so the extra iteration here is ok.
    const closest = vec3.create();
    getClosestPointOnDetailEdges(closest, tile, poly, polyIndex, pos, false);
    result.success = true;
    result.height = closest[1];
    return result;
};
/**
 * Get flags for edge in detail triangle.
 * @param[in]	triFlags		The flags for the triangle (last component of detail vertices above).
 * @param[in]	edgeIndex		The index of the first vertex of the edge. For instance, if 0,
 *								returns flags for edge AB.
 * @returns The edge flags
 */
const getDetailTriEdgeFlags = (triFlags, edgeIndex) => {
    return (triFlags >> (edgeIndex * 2)) & 0x3;
};
const _closestPointOnDetailEdgesTriangleVertices = [vec3.create(), vec3.create(), vec3.create()];
const _closestPointOnDetailEdgesPmin = vec3.create();
const _closestPointOnDetailEdgesPmax = vec3.create();
const _closestPointOnDetailEdges_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
/**
 * Gets the closest point on detail mesh edges to a given point
 * @param tile The tile containing the detail mesh
 * @param poly The polygon
 * @param pos The position to find closest point for
 * @param outClosestPoint Output parameter for the closest point
 * @param onlyBoundary If true, only consider boundary edges
 * @returns The squared distance to the closest point
 *  closest point
 */
const getClosestPointOnDetailEdges = (outClosestPoint, tile, poly, polyIndex, pos, onlyBoundary) => {
    const detailMesh = tile.detailMeshes[polyIndex];
    let dmin = Number.MAX_VALUE;
    let tmin = 0;
    const pmin = vec3.set(_closestPointOnDetailEdgesPmin, 0, 0, 0);
    const pmax = vec3.set(_closestPointOnDetailEdgesPmax, 0, 0, 0);
    for (let i = 0; i < detailMesh.trianglesCount; i++) {
        const t = (detailMesh.trianglesBase + i) * 4;
        const detailTriangles = tile.detailTriangles;
        // check if triangle has boundary edges (if onlyBoundary is true)
        if (onlyBoundary) {
            const triFlags = detailTriangles[t + 3];
            const ANY_BOUNDARY_EDGE = (DETAIL_EDGE_BOUNDARY << 0) | (DETAIL_EDGE_BOUNDARY << 2) | (DETAIL_EDGE_BOUNDARY << 4);
            if ((triFlags & ANY_BOUNDARY_EDGE) === 0) {
                continue;
            }
        }
        // get triangle vertices
        const triangleVertices = _closestPointOnDetailEdgesTriangleVertices;
        for (let j = 0; j < 3; ++j) {
            const vertexIndex = detailTriangles[t + j];
            if (vertexIndex < poly.vertices.length) {
                // use main polygon vertices - vertexIndex is an index into poly.vertices
                vec3.fromBuffer(triangleVertices[j], tile.vertices, poly.vertices[vertexIndex] * 3);
            }
            else {
                // use detail vertices - (vertexIndex - poly.vertices.length) gives offset from verticesBase
                const detailIndex = (detailMesh.verticesBase + (vertexIndex - poly.vertices.length)) * 3;
                vec3.fromBuffer(triangleVertices[j], tile.detailVertices, detailIndex);
            }
        }
        // check each edge of the triangle
        for (let k = 0, j = 2; k < 3; j = k++) {
            const triFlags = detailTriangles[t + 3];
            const edgeFlags = getDetailTriEdgeFlags(triFlags, j);
            // skip internal edges if we want only boundaries, or skip duplicate internal edges
            if ((edgeFlags & DETAIL_EDGE_BOUNDARY) === 0 && (onlyBoundary || detailTriangles[t + j] < detailTriangles[t + k])) {
                // only looking at boundary edges and this is internal, or
                // this is an inner edge that we will see again or have already seen.
                continue;
            }
            const result = distancePtSegSqr2d(_closestPointOnDetailEdges_distancePtSegSqr2dResult, pos, triangleVertices[j], triangleVertices[k]);
            if (result.distSqr < dmin) {
                dmin = result.distSqr;
                tmin = result.t;
                vec3.copy(pmin, triangleVertices[j]);
                vec3.copy(pmax, triangleVertices[k]);
            }
        }
    }
    // interpolate the final closest point
    if (pmin && pmax) {
        vec3.lerp(outClosestPoint, pmin, pmax, tmin);
    }
    return dmin;
};
const createGetClosestPointOnPolyResult = () => {
    return {
        success: false,
        isOverPoly: false,
        position: [0, 0, 0],
    };
};
const _getClosestPointOnPolyHeightResult = createGetPolyHeightResult();
/**
 * Gets the closest point on a polygon to a given point
 * @param result the result object to populate
 * @param navMesh the navigation mesh
 * @param nodeRef the polygon node reference
 * @param position the point to find the closest point to
 * @returns the result object
 */
const getClosestPointOnPoly = (result, navMesh, nodeRef, position) => {
    result.success = false;
    result.isOverPoly = false;
    vec3.copy(result.position, position);
    const tileAndPoly = getTileAndPolyByRef(nodeRef, navMesh);
    if (!tileAndPoly.success) {
        return result;
    }
    result.success = true;
    const { tile, poly, polyIndex } = tileAndPoly;
    const polyHeight = getPolyHeight(_getClosestPointOnPolyHeightResult, tile, poly, polyIndex, position);
    if (polyHeight.success) {
        vec3.copy(result.position, position);
        result.position[1] = polyHeight.height;
        result.isOverPoly = true;
        return result;
    }
    getClosestPointOnDetailEdges(result.position, tile, poly, polyIndex, position, true);
    return result;
};
const _closestPointOnPolyBoundaryLineStart = vec3.create();
const _closestPointOnPolyBoundaryLineEnd = vec3.create();
const _closestPointOnPolyBoundaryVertices = [];
const _closestPointOnPolyBoundary_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
/**
 * Gets the closest point on the boundary of a polygon to a given point
 * @param out the output closest point
 * @param navMesh the navigation mesh
 * @param nodeRef the polygon node reference
 * @param point the point to find the closest point to
 * @returns whether the operation was successful
 */
const getClosestPointOnPolyBoundary = (out, navMesh, nodeRef, point) => {
    const tileAndPoly = getTileAndPolyByRef(nodeRef, navMesh);
    if (!tileAndPoly.success || !vec3.finite(point) || !out) {
        return false;
    }
    const { tile, poly } = tileAndPoly;
    const lineStart = _closestPointOnPolyBoundaryLineStart;
    const lineEnd = _closestPointOnPolyBoundaryLineEnd;
    // collect vertices
    const verticesCount = poly.vertices.length;
    const vertices = _closestPointOnPolyBoundaryVertices;
    for (let i = 0; i < verticesCount; ++i) {
        const vIndex = poly.vertices[i] * 3;
        vertices[i * 3] = tile.vertices[vIndex];
        vertices[i * 3 + 1] = tile.vertices[vIndex + 1];
        vertices[i * 3 + 2] = tile.vertices[vIndex + 2];
    }
    // if inside polygon, return the point as-is
    if (pointInPoly(point, vertices, verticesCount)) {
        vec3.copy(out, point);
        return true;
    }
    // otherwise clamp to nearest edge
    let dmin = Number.MAX_VALUE;
    let imin = 0;
    for (let i = 0; i < verticesCount; ++i) {
        const j = (i + 1) % verticesCount;
        const vaIndex = i * 3;
        const vbIndex = j * 3;
        lineStart[0] = vertices[vaIndex + 0];
        lineStart[1] = vertices[vaIndex + 1];
        lineStart[2] = vertices[vaIndex + 2];
        lineEnd[0] = vertices[vbIndex + 0];
        lineEnd[1] = vertices[vbIndex + 1];
        lineEnd[2] = vertices[vbIndex + 2];
        distancePtSegSqr2d(_closestPointOnPolyBoundary_distancePtSegSqr2dResult, point, lineStart, lineEnd);
        if (_closestPointOnPolyBoundary_distancePtSegSqr2dResult.distSqr < dmin) {
            dmin = _closestPointOnPolyBoundary_distancePtSegSqr2dResult.distSqr;
            imin = i;
        }
    }
    const j = (imin + 1) % verticesCount;
    const vaIndex = imin * 3;
    const vbIndex = j * 3;
    const va0 = vertices[vaIndex + 0];
    const va1 = vertices[vaIndex + 1];
    const va2 = vertices[vaIndex + 2];
    const vb0 = vertices[vbIndex + 0];
    const vb1 = vertices[vbIndex + 1];
    const vb2 = vertices[vbIndex + 2];
    // compute t on segment (xz plane)
    const pqx = vb0 - va0;
    const pqz = vb2 - va2;
    const dx = point[0] - va0;
    const dz = point[2] - va2;
    const denom = pqx * pqx + pqz * pqz;
    let t = denom > 0 ? (pqx * dx + pqz * dz) / denom : 0;
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;
    out[0] = va0 + (vb0 - va0) * t;
    out[1] = va1 + (vb1 - va1) * t;
    out[2] = va2 + (vb2 - va2) * t;
    return true;
};
const createFindNearestPolyResult = () => {
    return {
        success: false,
        nodeRef: 0,
        position: [0, 0, 0],
    };
};
const _findNearestPolyClosestPointResult = createGetClosestPointOnPolyResult();
const _findNearestPolyDiff = vec3.create();
const _findNearestPolyBounds = box3.create();
const findNearestPoly = (result, navMesh, center, halfExtents, queryFilter) => {
    result.success = false;
    result.nodeRef = 0;
    vec3.copy(result.position, center);
    // get bounds for the query
    const bounds = _findNearestPolyBounds;
    vec3.sub(bounds[0], center, halfExtents);
    vec3.add(bounds[1], center, halfExtents);
    // query polygons within the query bounds
    const polys = queryPolygons(navMesh, bounds, queryFilter);
    let nearestDistSqr = Number.MAX_VALUE;
    // find the closest polygon
    for (const ref of polys) {
        const closestPoint = getClosestPointOnPoly(_findNearestPolyClosestPointResult, navMesh, ref, center);
        if (!closestPoint.success)
            continue;
        const { tileId } = getNodeByRef(navMesh, ref);
        const tile = navMesh.tiles[tileId];
        if (!tile)
            continue;
        // calculate difference vector
        vec3.sub(_findNearestPolyDiff, center, closestPoint.position);
        let distSqr;
        // if a point is directly over a polygon and closer than
        // climb height, favor that instead of straight line nearest point.
        if (closestPoint.isOverPoly) {
            const heightDiff = Math.abs(_findNearestPolyDiff[1]) - tile.walkableClimb;
            distSqr = heightDiff > 0 ? heightDiff * heightDiff : 0;
        }
        else {
            distSqr = vec3.squaredLength(_findNearestPolyDiff);
        }
        if (distSqr < nearestDistSqr) {
            nearestDistSqr = distSqr;
            result.nodeRef = ref;
            vec3.copy(result.position, closestPoint.position);
            result.success = true;
        }
    }
    return result;
};
const _queryPolygonsInTileBmax = vec3.create();
const _queryPolygonsInTileBmin = vec3.create();
const queryPolygonsInTile = (out, navMesh, tile, bounds, filter) => {
    const qmin = bounds[0];
    const qmax = bounds[1];
    let nodeIndex = 0;
    const endIndex = tile.bvTree.nodes.length;
    const tbmin = tile.bounds[0];
    const tbmax = tile.bounds[1];
    const qfac = tile.bvTree.quantFactor;
    // clamp query box to world box.
    const minx = Math.max(Math.min(qmin[0], tbmax[0]), tbmin[0]) - tbmin[0];
    const miny = Math.max(Math.min(qmin[1], tbmax[1]), tbmin[1]) - tbmin[1];
    const minz = Math.max(Math.min(qmin[2], tbmax[2]), tbmin[2]) - tbmin[2];
    const maxx = Math.max(Math.min(qmax[0], tbmax[0]), tbmin[0]) - tbmin[0];
    const maxy = Math.max(Math.min(qmax[1], tbmax[1]), tbmin[1]) - tbmin[1];
    const maxz = Math.max(Math.min(qmax[2], tbmax[2]), tbmin[2]) - tbmin[2];
    // quantize
    _queryPolygonsInTileBmin[0] = Math.floor(qfac * minx) & 0xfffe;
    _queryPolygonsInTileBmin[1] = Math.floor(qfac * miny) & 0xfffe;
    _queryPolygonsInTileBmin[2] = Math.floor(qfac * minz) & 0xfffe;
    _queryPolygonsInTileBmax[0] = Math.floor(qfac * maxx + 1) | 1;
    _queryPolygonsInTileBmax[1] = Math.floor(qfac * maxy + 1) | 1;
    _queryPolygonsInTileBmax[2] = Math.floor(qfac * maxz + 1) | 1;
    // traverse tree
    while (nodeIndex < endIndex) {
        const bvNode = tile.bvTree.nodes[nodeIndex];
        const nodeBounds = bvNode.bounds;
        const overlap = _queryPolygonsInTileBmin[0] <= nodeBounds[1][0] &&
            _queryPolygonsInTileBmax[0] >= nodeBounds[0][0] &&
            _queryPolygonsInTileBmin[1] <= nodeBounds[1][1] &&
            _queryPolygonsInTileBmax[1] >= nodeBounds[0][1] &&
            _queryPolygonsInTileBmin[2] <= nodeBounds[1][2] &&
            _queryPolygonsInTileBmax[2] >= nodeBounds[0][2];
        const isLeafNode = bvNode.i >= 0;
        if (isLeafNode && overlap) {
            const polyIndex = bvNode.i;
            const node = getNodeByTileAndPoly(navMesh, tile, polyIndex);
            if (filter.passFilter(node.ref, navMesh)) {
                out.push(node.ref);
            }
        }
        if (overlap || isLeafNode) {
            nodeIndex++;
        }
        else {
            const escapeIndex = -bvNode.i;
            nodeIndex += escapeIndex;
        }
    }
};
const _queryPolygonsMinTile = vec2.create();
const _queryPolygonsMaxTile = vec2.create();
const queryPolygons = (navMesh, bounds, filter) => {
    const result = [];
    // find min and max tile positions
    const minTile = worldToTilePosition(_queryPolygonsMinTile, navMesh, bounds[0]);
    const maxTile = worldToTilePosition(_queryPolygonsMaxTile, navMesh, bounds[1]);
    // iterate through the tiles in the query bounds
    if (!vec2.finite(minTile) || !vec2.finite(maxTile)) {
        return result;
    }
    for (let x = minTile[0]; x <= maxTile[0]; x++) {
        for (let y = minTile[1]; y <= maxTile[1]; y++) {
            const tiles = getTilesAt(navMesh, x, y);
            for (const tile of tiles) {
                queryPolygonsInTile(result, navMesh, tile, bounds, filter);
            }
        }
    }
    return result;
};
const allocateNode = (navMesh) => {
    const nodeIndex = requestIndex(navMesh.nodeIndexPool);
    let node = navMesh.nodes[nodeIndex];
    if (!node) {
        node = navMesh.nodes[nodeIndex] = {
            allocated: true,
            index: nodeIndex,
            ref: 0,
            area: 0,
            flags: 0,
            links: [],
            type: 0,
            tileId: -1,
            polyIndex: -1,
            offMeshConnectionId: -1,
        };
    }
    node.allocated = true;
    return node;
};
const releaseNode = (navMesh, index) => {
    const node = navMesh.nodes[index];
    node.allocated = false;
    node.links.length = 0;
    node.ref = 0;
    node.type = 0;
    node.area = -1;
    node.flags = -1;
    node.tileId = -1;
    node.polyIndex = -1;
    node.offMeshConnectionId = -1;
    releaseIndex(navMesh.nodeIndexPool, index);
};
/**
 * Allocates a link and returns it's index
 */
const allocateLink = (navMesh) => {
    const linkIndex = requestIndex(navMesh.linkIndexPool);
    let link = navMesh.links[linkIndex];
    if (!link) {
        link = navMesh.links[linkIndex] = {
            allocated: true,
            index: linkIndex,
            fromNodeIndex: 0,
            fromNodeRef: 0,
            toNodeIndex: 0,
            toNodeRef: 0,
            edge: 0,
            side: 0,
            bmin: 0,
            bmax: 0,
        };
    }
    link.allocated = true;
    return linkIndex;
};
/**
 * Releases a link
 */
const releaseLink = (navMesh, index) => {
    const link = navMesh.links[index];
    link.allocated = false;
    link.fromNodeIndex = -1;
    link.fromNodeRef = 0;
    link.toNodeIndex = -1;
    link.toNodeRef = 0;
    link.edge = 0;
    link.side = 0;
    link.bmin = 0;
    link.bmax = 0;
    releaseIndex(navMesh.linkIndexPool, index);
};
const connectInternalLinks = (navMesh, tile) => {
    // create links between polygons within the tile
    // based on the neighbor information stored in each polygon
    for (let polyIndex = 0; polyIndex < tile.polys.length; polyIndex++) {
        const poly = tile.polys[polyIndex];
        const node = getNodeByTileAndPoly(navMesh, tile, polyIndex);
        for (let edgeIndex = 0; edgeIndex < poly.vertices.length; edgeIndex++) {
            const neiValue = poly.neis[edgeIndex];
            // skip external links and border edges
            if (neiValue === 0 || neiValue & POLY_NEIS_FLAG_EXT_LINK) {
                continue;
            }
            // internal connection - create link
            const neighborPolyIndex = neiValue - 1; // convert back to 0-based indexing
            if (neighborPolyIndex >= 0 && neighborPolyIndex < tile.polys.length) {
                const linkIndex = allocateLink(navMesh);
                const link = navMesh.links[linkIndex];
                const neighbourNode = getNodeByTileAndPoly(navMesh, tile, neighborPolyIndex);
                link.fromNodeIndex = node.index;
                link.fromNodeRef = node.ref;
                link.toNodeIndex = neighbourNode.index;
                link.toNodeRef = neighbourNode.ref;
                link.edge = edgeIndex; // edge index in current polygon
                link.side = 0xff; // not a boundary link
                link.bmin = 0; // not used for internal links
                link.bmax = 0; // not used for internal links
                node.links.push(linkIndex);
            }
        }
    }
};
const oppositeTile = (side) => (side + 4) & 0x7;
// Compute a scalar coordinate along the primary axis for the slab
const getSlabCoord = (v, side) => {
    if (side === 0 || side === 4)
        return v[0]; // x portals measure by x
    if (side === 2 || side === 6)
        return v[2]; // z portals measure by z
    return 0;
};
// Calculate 2D endpoints (u,y) for edge segment projected onto the portal axis plane.
// For x-portals (side 0/4) we use u = z, for z-portals (2/6) u = x.
const calcSlabEndPoints = (va, vb, bmin, bmax, side) => {
    if (side === 0 || side === 4) {
        if (va[2] < vb[2]) {
            bmin[0] = va[2];
            bmin[1] = va[1];
            bmax[0] = vb[2];
            bmax[1] = vb[1];
        }
        else {
            bmin[0] = vb[2];
            bmin[1] = vb[1];
            bmax[0] = va[2];
            bmax[1] = va[1];
        }
    }
    else if (side === 2 || side === 6) {
        if (va[0] < vb[0]) {
            bmin[0] = va[0];
            bmin[1] = va[1];
            bmax[0] = vb[0];
            bmax[1] = vb[1];
        }
        else {
            bmin[0] = vb[0];
            bmin[1] = vb[1];
            bmax[0] = va[0];
            bmax[1] = va[1];
        }
    }
};
// Overlap test of two edge slabs in (u,y) space, with tolerances px (horizontal pad) and py (vertical threshold)
const overlapSlabs = (amin, amax, bmin, bmax, px, py) => {
    const minx = Math.max(amin[0] + px, bmin[0] + px);
    const maxx = Math.min(amax[0] - px, bmax[0] - px);
    if (minx > maxx)
        return false; // no horizontal overlap
    // Vertical overlap test via line interpolation along u
    const ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
    const ak = amin[1] - ad * amin[0];
    const bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
    const bk = bmin[1] - bd * bmin[0];
    const aminy = ad * minx + ak;
    const amaxy = ad * maxx + ak;
    const bminy = bd * minx + bk;
    const bmaxy = bd * maxx + bk;
    const dmin = bminy - aminy;
    const dmax = bmaxy - amaxy;
    if (dmin * dmax < 0)
        return true; // crossing
    const thr = py * 2 * (py * 2);
    if (dmin * dmin <= thr || dmax * dmax <= thr)
        return true; // near endpoints
    return false;
};
const _amin = vec3.create();
const _amax = vec3.create();
const _bmin = vec3.create();
const _bmax = vec3.create();
/**
 * Find connecting external polys between edge va->vb in target tile on opposite side.
 * Returns array of { ref, tmin, tmax } describing overlapping intervals along the edge.
 * @param va vertex A
 * @param vb vertex B
 * @param target target tile
 * @param side portal side
 * @returns array of connecting polygons
 */
const findConnectingPolys = (navMesh, va, vb, target, side) => {
    if (!target)
        return [];
    calcSlabEndPoints(va, vb, _amin, _amax, side); // store u,y
    const apos = getSlabCoord(va, side);
    const results = [];
    // iterate target polys & their boundary edges (those marked ext link in that direction)
    for (let i = 0; i < target.polys.length; i++) {
        const poly = target.polys[i];
        const nv = poly.vertices.length;
        for (let j = 0; j < nv; j++) {
            const nei = poly.neis[j];
            // not an external edge
            if ((nei & POLY_NEIS_FLAG_EXT_LINK) === 0)
                continue;
            const dir = nei & POLY_NEIS_FLAG_EXT_LINK_DIR_MASK;
            // only edges that face the specified side from target perspective
            if (dir !== side)
                continue;
            const vcIndex = poly.vertices[j];
            const vdIndex = poly.vertices[(j + 1) % nv];
            const vc = [target.vertices[vcIndex * 3], target.vertices[vcIndex * 3 + 1], target.vertices[vcIndex * 3 + 2]];
            const vd = [target.vertices[vdIndex * 3], target.vertices[vdIndex * 3 + 1], target.vertices[vdIndex * 3 + 2]];
            const bpos = getSlabCoord(vc, side);
            // not co-planar enough
            if (Math.abs(apos - bpos) > 0.01)
                continue;
            calcSlabEndPoints(vc, vd, _bmin, _bmax, side);
            if (!overlapSlabs(_amin, _amax, _bmin, _bmax, 0.01, target.walkableClimb))
                continue;
            // record overlap interval
            const polyRef = getNodeByTileAndPoly(navMesh, target, i).ref;
            results.push({
                nodeRef: polyRef,
                umin: Math.max(_amin[0], _bmin[0]),
                umax: Math.min(_amax[0], _bmax[0]),
            });
            // proceed to next polygon (edge matched)
            break;
        }
    }
    return results;
};
const _va = vec3.create();
const _vb = vec3.create();
const connectExternalLinks = (navMesh, tile, target, side) => {
    // connect border links
    for (let polyIndex = 0; polyIndex < tile.polys.length; polyIndex++) {
        const poly = tile.polys[polyIndex];
        // get the node for this poly
        const node = getNodeByTileAndPoly(navMesh, tile, polyIndex);
        const nv = poly.vertices.length;
        for (let j = 0; j < nv; j++) {
            // skip non-portal edges
            if ((poly.neis[j] & POLY_NEIS_FLAG_EXT_LINK) === 0) {
                continue;
            }
            const dir = poly.neis[j] & POLY_NEIS_FLAG_EXT_LINK_DIR_MASK;
            if (side !== -1 && dir !== side) {
                continue;
            }
            // create new links
            const va = vec3.fromBuffer(_va, tile.vertices, poly.vertices[j] * 3);
            const vb = vec3.fromBuffer(_vb, tile.vertices, poly.vertices[(j + 1) % nv] * 3);
            // find overlaps against target tile along the opposite side direction
            const overlaps = findConnectingPolys(navMesh, va, vb, target, oppositeTile(dir));
            for (const o of overlaps) {
                // parameterize overlap interval along this edge to [0,1]
                let tmin;
                let tmax;
                if (dir === 0 || dir === 4) {
                    // x portals param by z
                    tmin = (o.umin - va[2]) / (vb[2] - va[2]);
                    tmax = (o.umax - va[2]) / (vb[2] - va[2]);
                }
                else {
                    // z portals param by x
                    tmin = (o.umin - va[0]) / (vb[0] - va[0]);
                    tmax = (o.umax - va[0]) / (vb[0] - va[0]);
                }
                if (tmin > tmax) {
                    const tmp = tmin;
                    tmin = tmax;
                    tmax = tmp;
                }
                tmin = Math.max(0, Math.min(1, tmin));
                tmax = Math.max(0, Math.min(1, tmax));
                const linkIndex = allocateLink(navMesh);
                const link = navMesh.links[linkIndex];
                link.fromNodeIndex = node.index;
                link.fromNodeRef = node.ref;
                link.toNodeIndex = getNodeRefIndex(o.nodeRef);
                link.toNodeRef = o.nodeRef;
                link.edge = j;
                link.side = dir;
                link.bmin = Math.round(tmin * 255);
                link.bmax = Math.round(tmax * 255);
                node.links.push(linkIndex);
            }
        }
    }
};
/**
 * Disconnect external links from tile to target tile
 */
const disconnectExternalLinks = (navMesh, tile, target) => {
    const targetId = target.id;
    for (let polyIndex = 0; polyIndex < tile.polys.length; polyIndex++) {
        const node = getNodeByTileAndPoly(navMesh, tile, polyIndex);
        const filteredLinks = [];
        for (let k = 0; k < node.links.length; k++) {
            const linkIndex = node.links[k];
            const link = navMesh.links[linkIndex];
            const neiNode = getNodeByRef(navMesh, link.toNodeRef);
            if (neiNode.tileId === targetId) {
                releaseLink(navMesh, linkIndex);
            }
            else {
                filteredLinks.push(linkIndex);
            }
        }
        node.links = filteredLinks;
    }
};
const createOffMeshLink = (navMesh, from, to, edge) => {
    const linkIndex = allocateLink(navMesh);
    const link = navMesh.links[linkIndex];
    link.fromNodeIndex = from.index;
    link.fromNodeRef = from.ref;
    link.toNodeIndex = to.index;
    link.toNodeRef = to.ref;
    link.edge = edge;
    link.side = 0; // not used for offmesh links
    link.bmin = 0; // not used for offmesh links
    link.bmax = 0; // not used for offmesh links
    from.links.push(linkIndex);
};
const _connectOffMeshConnection_nearestPolyStart = createFindNearestPolyResult();
const _connectOffMeshConnection_nearestPolyEnd = createFindNearestPolyResult();
const _connectOffMeshConnection_halfExtents = vec3.create();
const connectOffMeshConnection = (navMesh, offMeshConnection) => {
    // find polys for the start and end positions
    const radiusHalfExtents = vec3.set(_connectOffMeshConnection_halfExtents, offMeshConnection.radius, offMeshConnection.radius, offMeshConnection.radius);
    const startTilePolyResult = findNearestPoly(_connectOffMeshConnection_nearestPolyStart, navMesh, offMeshConnection.start, radiusHalfExtents, DEFAULT_QUERY_FILTER);
    const endTilePolyResult = findNearestPoly(_connectOffMeshConnection_nearestPolyEnd, navMesh, offMeshConnection.end, radiusHalfExtents, DEFAULT_QUERY_FILTER);
    // exit if we couldn't find a start or an end poly, can't connect off mesh connection
    if (!startTilePolyResult.success || !endTilePolyResult.success) {
        return false;
    }
    // get start and end poly nodes
    const startNodeRef = startTilePolyResult.nodeRef;
    const startNode = getNodeByRef(navMesh, startNodeRef);
    const endNodeRef = endTilePolyResult.nodeRef;
    const endNode = getNodeByRef(navMesh, endNodeRef);
    // create off mesh connection state, for quick revalidation of connections when adding and removing tiles
    const offMeshConnectionState = {
        offMeshNode: -1,
        startPolyNode: startNodeRef,
        endPolyNode: endNodeRef,
    };
    navMesh.offMeshConnectionAttachments[offMeshConnection.id] = offMeshConnectionState;
    // create a node for the off mesh connection
    const offMeshNode = allocateNode(navMesh);
    const offMeshNodeRef = serNodeRef(NodeType.OFFMESH, offMeshNode.index, offMeshConnection.sequence);
    offMeshNode.type = NodeType.OFFMESH;
    offMeshNode.ref = offMeshNodeRef;
    offMeshNode.area = offMeshConnection.area;
    offMeshNode.flags = offMeshConnection.flags;
    offMeshNode.offMeshConnectionId = offMeshConnection.id;
    offMeshConnectionState.offMeshNode = offMeshNodeRef;
    // start poly -> off mesh node -> end poly
    createOffMeshLink(navMesh, startNode, offMeshNode, 0);
    createOffMeshLink(navMesh, offMeshNode, endNode, 1);
    if (offMeshConnection.direction === OffMeshConnectionDirection.BIDIRECTIONAL) {
        // end poly -> off mesh node -> start poly
        createOffMeshLink(navMesh, endNode, offMeshNode, 1);
        createOffMeshLink(navMesh, offMeshNode, startNode, 0);
    }
    // connected the off mesh connection, return true
    return true;
};
const disconnectOffMeshConnection = (navMesh, offMeshConnection) => {
    const offMeshConnectionState = navMesh.offMeshConnectionAttachments[offMeshConnection.id];
    // the off mesh connection is not connected, return false
    if (!offMeshConnectionState)
        return false;
    const offMeshConnectionNodeRef = offMeshConnectionState.offMeshNode;
    const startPolyNode = offMeshConnectionState.startPolyNode;
    const endPolyNode = offMeshConnectionState.endPolyNode;
    // release links in the start and end polys that reference off mesh connection nodes
    const startNode = getNodeByRef(navMesh, startPolyNode);
    if (startNode) {
        for (let i = startNode.links.length - 1; i >= 0; i--) {
            const linkId = startNode.links[i];
            const link = navMesh.links[linkId];
            if (link.toNodeRef === offMeshConnectionNodeRef) {
                releaseLink(navMesh, linkId);
                startNode.links.splice(i, 1);
            }
        }
    }
    const endNode = getNodeByRef(navMesh, endPolyNode);
    if (endNode) {
        for (let i = endNode.links.length - 1; i >= 0; i--) {
            const linkId = endNode.links[i];
            const link = navMesh.links[linkId];
            if (link.toNodeRef === offMeshConnectionNodeRef) {
                releaseLink(navMesh, linkId);
                endNode.links.splice(i, 1);
            }
        }
    }
    // release the off mesh node and links
    const offMeshNode = getNodeByRef(navMesh, offMeshConnectionNodeRef);
    if (offMeshNode) {
        for (let i = offMeshNode.links.length - 1; i >= 0; i--) {
            const linkId = offMeshNode.links[i];
            releaseLink(navMesh, linkId);
        }
    }
    releaseNode(navMesh, getNodeRefIndex(offMeshConnectionNodeRef));
    // remove the off mesh connection state
    delete navMesh.offMeshConnectionAttachments[offMeshConnection.id];
    // the off mesh connection was disconnected, return true
    return true;
};
/**
 * Reconnects an off mesh connection. This must be called if any properties of an off mesh connection are changed, for example the start or end positions.
 * @param navMesh the navmesh
 * @param offMeshConnection the off mesh connectionion to reconnect
 * @returns whether the off mesh connection was successfully reconnected
 */
const reconnectOffMeshConnection = (navMesh, offMeshConnection) => {
    disconnectOffMeshConnection(navMesh, offMeshConnection);
    return connectOffMeshConnection(navMesh, offMeshConnection);
};
const updateOffMeshConnections = (navMesh) => {
    for (const id in navMesh.offMeshConnections) {
        const offMeshConnection = navMesh.offMeshConnections[id];
        const connected = isOffMeshConnectionConnected(navMesh, offMeshConnection.id);
        if (!connected) {
            reconnectOffMeshConnection(navMesh, offMeshConnection);
        }
    }
};
/**
 * Builds a navmesh tile from the given parameters
 * This builds a BV-tree for the tile, and initializes runtime tile properties
 * @param params the parameters to build the tile from
 * @returns the built navmesh tile
 */
const buildTile = (params) => {
    const bvTree = buildNavMeshBvTree(params);
    const tile = {
        ...params,
        id: -1,
        sequence: -1,
        bvTree,
        polyNodes: [],
    };
    return tile;
};
/**
 * Adds a tile to the navmesh.
 * If a tile already exists at the same position, it will be removed first.
 * @param navMesh the navmesh to add the tile to
 * @param tile the tile to add
 */
const addTile = (navMesh, tile) => {
    const tilePositionHash = getTilePositionHash(tile.tileX, tile.tileY, tile.tileLayer);
    // remove any existing tile at the same position
    if (navMesh.tilePositionToTileId[tilePositionHash] !== undefined) {
        removeTile(navMesh, tile.tileX, tile.tileY, tile.tileLayer);
    }
    // tile sequence
    let sequence = navMesh.tilePositionToSequenceCounter[tilePositionHash];
    if (sequence === undefined) {
        sequence = 0;
    }
    else {
        sequence = (sequence + 1) % MAX_SEQUENCE;
    }
    navMesh.tilePositionToSequenceCounter[tilePositionHash] = sequence;
    // get tile id
    const id = requestIndex(navMesh.tileIndexPool);
    // set tile id and sequence
    tile.id = id;
    tile.sequence = sequence;
    // store tile in navmesh
    navMesh.tiles[tile.id] = tile;
    // store position lookup
    navMesh.tilePositionToTileId[tilePositionHash] = tile.id;
    // store column lookup
    const tileColumnHash = getTileColumnHash(tile.tileX, tile.tileY);
    if (!navMesh.tileColumnToTileIds[tileColumnHash]) {
        navMesh.tileColumnToTileIds[tileColumnHash] = [];
    }
    navMesh.tileColumnToTileIds[tileColumnHash].push(tile.id);
    // allocate nodes
    for (let i = 0; i < tile.polys.length; i++) {
        const node = allocateNode(navMesh);
        node.ref = serNodeRef(NodeType.POLY, node.index, tile.sequence);
        node.type = NodeType.POLY;
        node.area = tile.polys[i].area;
        node.flags = tile.polys[i].flags;
        node.tileId = tile.id;
        node.polyIndex = i;
        node.links.length = 0;
        tile.polyNodes.push(node.index);
    }
    // create internal links within the tile
    connectInternalLinks(navMesh, tile);
    // connect with layers in current tile.
    const tilesAtCurrentPosition = getTilesAt(navMesh, tile.tileX, tile.tileY);
    for (const tileAtCurrentPosition of tilesAtCurrentPosition) {
        if (tileAtCurrentPosition.id === tile.id)
            continue;
        connectExternalLinks(navMesh, tileAtCurrentPosition, tile, -1);
        connectExternalLinks(navMesh, tile, tileAtCurrentPosition, -1);
    }
    // connect with neighbouring tiles
    for (let side = 0; side < 8; side++) {
        const neighbourTiles = getNeighbourTilesAt(navMesh, tile.tileX, tile.tileY, side);
        for (const neighbourTile of neighbourTiles) {
            connectExternalLinks(navMesh, tile, neighbourTile, side);
            connectExternalLinks(navMesh, neighbourTile, tile, oppositeTile(side));
        }
    }
    // update off mesh connections
    updateOffMeshConnections(navMesh);
};
/**
 * Removes the tile at the given location
 * @param navMesh the navmesh to remove the tile from
 * @param x the x coordinate of the tile
 * @param y the y coordinate of the tile
 * @param layer the layer of the tile
 * @returns true if the tile was removed, otherwise false
 */
const removeTile = (navMesh, x, y, layer) => {
    const tileHash = getTilePositionHash(x, y, layer);
    const tileId = navMesh.tilePositionToTileId[tileHash];
    const tile = navMesh.tiles[tileId];
    if (!tile) {
        return false;
    }
    // disconnect external links with tiles in the same layer
    const tilesAtCurrentPosition = getTilesAt(navMesh, x, y);
    for (const tileAtCurrentPosition of tilesAtCurrentPosition) {
        if (tileAtCurrentPosition.id === tileId)
            continue;
        disconnectExternalLinks(navMesh, tileAtCurrentPosition, tile);
        disconnectExternalLinks(navMesh, tile, tileAtCurrentPosition);
    }
    // disconnect external links with neighbouring tiles
    for (let side = 0; side < 8; side++) {
        const neighbourTiles = getNeighbourTilesAt(navMesh, x, y, side);
        for (const neighbourTile of neighbourTiles) {
            disconnectExternalLinks(navMesh, neighbourTile, tile);
            disconnectExternalLinks(navMesh, tile, neighbourTile);
        }
    }
    // release internal links
    for (let polyIndex = 0; polyIndex < tile.polys.length; polyIndex++) {
        const node = getNodeByTileAndPoly(navMesh, tile, polyIndex);
        for (const link of node.links) {
            releaseLink(navMesh, link);
        }
    }
    // release nodes
    for (let i = 0; i < tile.polyNodes.length; i++) {
        releaseNode(navMesh, tile.polyNodes[i]);
    }
    tile.polyNodes.length = 0;
    // remove tile from navmesh
    delete navMesh.tiles[tileId];
    // remove position lookup
    delete navMesh.tilePositionToTileId[tileHash];
    // remove column lookup
    const tileColumnHash = getTileColumnHash(x, y);
    const tileColumn = navMesh.tileColumnToTileIds[tileColumnHash];
    if (tileColumn) {
        const tileIndexInColumn = tileColumn.indexOf(tileId);
        if (tileIndexInColumn !== -1) {
            tileColumn.splice(tileIndexInColumn, 1);
        }
        if (tileColumn.length === 0) {
            delete navMesh.tileColumnToTileIds[tileColumnHash];
        }
    }
    // release tile index to the pool
    releaseIndex(navMesh.tileIndexPool, tileId);
    // update off mesh connections
    updateOffMeshConnections(navMesh);
    return true;
};
/**
 * Adds a new off mesh connection to the NavMesh, and returns it's ID
 * @param navMesh the navmesh to add the off mesh connection to
 * @param offMeshConnectionParams the parameters of the off mesh connection to add
 * @returns the ID of the added off mesh connection
 */
const addOffMeshConnection = (navMesh, offMeshConnectionParams) => {
    const id = requestIndex(navMesh.offMeshConnectionIndexPool);
    const sequence = navMesh.offMeshConnectionSequenceCounter;
    navMesh.offMeshConnectionSequenceCounter = (navMesh.offMeshConnectionSequenceCounter + 1) % MAX_SEQUENCE;
    const offMeshConnection = {
        ...offMeshConnectionParams,
        id,
        sequence,
    };
    navMesh.offMeshConnections[id] = offMeshConnection;
    connectOffMeshConnection(navMesh, offMeshConnection);
    return id;
};
/**
 * Removes an off mesh connection from the NavMesh
 * @param navMesh the navmesh to remove the off mesh connection from
 * @param offMeshConnectionId the ID of the off mesh connection to remove
 */
const removeOffMeshConnection = (navMesh, offMeshConnectionId) => {
    const offMeshConnection = navMesh.offMeshConnections[offMeshConnectionId];
    if (!offMeshConnection)
        return;
    releaseIndex(navMesh.offMeshConnectionIndexPool, offMeshConnection.id);
    disconnectOffMeshConnection(navMesh, offMeshConnection);
    delete navMesh.offMeshConnections[offMeshConnection.id];
};
/**
 * Returns whether the off mesh connection with the given ID is currently connected to the navmesh.
 * An off mesh connection may be disconnected if the start or end positions have no valid polygons nearby to connect to.
 * @param navMesh the navmesh
 * @param offMeshConnectionId the ID of the off mesh connection
 * @returns whether the off mesh connection is connected
 */
const isOffMeshConnectionConnected = (navMesh, offMeshConnectionId) => {
    const offMeshConnectionState = navMesh.offMeshConnectionAttachments[offMeshConnectionId];
    // no off mesh connection state, not connected
    if (!offMeshConnectionState)
        return false;
    const { startPolyNode, endPolyNode } = offMeshConnectionState;
    // valid if both the start and end poly node refs are valid
    return isValidNodeRef(navMesh, startPolyNode) && isValidNodeRef(navMesh, endPolyNode);
};
const ANY_QUERY_FILTER = {
    getCost(pa, pb, _navMesh, _prevRef, _curRef, _nextRef) {
        // use the distance between the two points as the cost
        return vec3.distance(pa, pb);
    },
    passFilter(_nodeRef, _navMesh) {
        return true;
    },
};
const createDefaultQueryFilter = () => {
    return {
        includeFlags: 0xffffffff,
        excludeFlags: 0,
        getCost(pa, pb, _navMesh, _prevRef, _curRef, _nextRef) {
            // use the distance between the two points as the cost
            return vec3.distance(pa, pb);
        },
        passFilter(nodeRef, navMesh) {
            // check whether the node's flags pass 'includeFlags' and 'excludeFlags' checks
            const { flags } = getNodeByRef(navMesh, nodeRef);
            return (flags & this.includeFlags) !== 0 && (flags & this.excludeFlags) === 0;
        },
    };
};
const DEFAULT_QUERY_FILTER = createDefaultQueryFilter();

const NODE_FLAG_OPEN = 0x01;
const NODE_FLAG_CLOSED = 0x02;
/** parent of the node is not adjacent. Found using raycast. */
const NODE_FLAG_PARENT_DETACHED = 0x04;
const getSearchNode = (pool, nodeRef, state) => {
    const nodes = pool[nodeRef];
    if (!nodes)
        return undefined;
    for (let i = 0; i < nodes.length; i++) {
        if (nodes[i].state === state) {
            return nodes[i];
        }
    }
    return undefined;
};
const addSearchNode = (pool, node) => {
    if (!pool[node.nodeRef]) {
        pool[node.nodeRef] = [];
    }
    pool[node.nodeRef].push(node);
};
const bubbleUpQueue = (queue, i, node) => {
    // note: (index > 0) means there is a parent
    let parent = Math.floor((i - 1) / 2);
    while (i > 0 && queue[parent].total > node.total) {
        queue[i] = queue[parent];
        i = parent;
        parent = Math.floor((i - 1) / 2);
    }
    queue[i] = node;
};
const trickleDownQueue = (queue, i, node) => {
    const count = queue.length;
    let child = 2 * i + 1;
    while (child < count) {
        // if there is a right child and it is smaller than the left child
        if (child + 1 < count && queue[child + 1].total < queue[child].total) {
            child++;
        }
        // if the current node is smaller than the smallest child, we are done
        if (node.total <= queue[child].total) {
            break;
        }
        // move the smallest child up
        queue[i] = queue[child];
        i = child;
        child = i * 2 + 1;
    }
    queue[i] = node;
};
const pushNodeToQueue = (queue, node) => {
    queue.push(node);
    bubbleUpQueue(queue, queue.length - 1, node);
};
const popNodeFromQueue = (queue) => {
    if (queue.length === 0) {
        return undefined;
    }
    const node = queue[0];
    const lastNode = queue.pop();
    if (queue.length > 0 && lastNode !== undefined) {
        queue[0] = lastNode;
        trickleDownQueue(queue, 0, lastNode);
    }
    return node;
};
const reindexNodeInQueue = (queue, node) => {
    for (let i = 0; i < queue.length; i++) {
        if (queue[i].nodeRef === node.nodeRef && queue[i].state === node.state) {
            queue[i] = node;
            bubbleUpQueue(queue, i, node);
            return;
        }
    }
};
const _getPortalPoints_start = vec3.create();
const _getPortalPoints_end = vec3.create();
/**
 * Retrieves the left and right points of the portal edge between two adjacent polygons.
 * Or if one of the polygons is an off-mesh connection, returns the connection endpoint for both left and right.
 */
const getPortalPoints = (navMesh, fromNodeRef, toNodeRef, outLeft, outRight) => {
    // find the link that points to the 'to' polygon.
    let toLink;
    const fromNode = getNodeByRef(navMesh, fromNodeRef);
    for (const linkIndex of fromNode.links) {
        const link = navMesh.links[linkIndex];
        if (link.toNodeRef === toNodeRef) {
            // found the link to the target polygon.
            toLink = link;
            break;
        }
    }
    if (!toLink) {
        // no link found to the target polygon.
        return false;
    }
    const fromNodeType = getNodeRefType(fromNodeRef);
    const toNodeType = getNodeRefType(toNodeRef);
    // handle from poly to poly
    if (fromNodeType === NodeType.POLY && toNodeType === NodeType.POLY) {
        // get the 'from' and 'to' tiles
        const { tileId: fromTileId, polyIndex: fromPolyIndex } = getNodeByRef(navMesh, fromNodeRef);
        const fromTile = navMesh.tiles[fromTileId];
        const fromPoly = fromTile.polys[fromPolyIndex];
        // find portal vertices
        const v0Index = fromPoly.vertices[toLink.edge];
        const v1Index = fromPoly.vertices[(toLink.edge + 1) % fromPoly.vertices.length];
        const v0Offset = v0Index * 3;
        const v1Offset = v1Index * 3;
        // if the link is at tile boundary, clamp the vertices to the link width.
        if (toLink.side !== 0xff && (toLink.bmin !== 0 || toLink.bmax !== 255)) {
            // unpack portal limits and lerp
            const s = 1.0 / 255.0;
            const tmin = toLink.bmin * s;
            const tmax = toLink.bmax * s;
            vec3.fromBuffer(_getPortalPoints_start, fromTile.vertices, v0Offset);
            vec3.fromBuffer(_getPortalPoints_end, fromTile.vertices, v1Offset);
            vec3.lerp(outLeft, _getPortalPoints_start, _getPortalPoints_end, tmin);
            vec3.lerp(outRight, _getPortalPoints_start, _getPortalPoints_end, tmax);
        }
        else {
            // no clamping needed - direct copy
            vec3.fromBuffer(outLeft, fromTile.vertices, v0Offset);
            vec3.fromBuffer(outRight, fromTile.vertices, v1Offset);
        }
        return true;
    }
    // handle from poly to offmesh connection
    if (fromNodeType === NodeType.POLY && toNodeType === NodeType.OFFMESH) {
        const toNode = getNodeByRef(navMesh, toNodeRef);
        const offMeshConnection = navMesh.offMeshConnections[toNode.offMeshConnectionId];
        if (!offMeshConnection)
            return false;
        const position = toLink && toLink.edge === 0 ? offMeshConnection.start : offMeshConnection.end;
        vec3.copy(outLeft, position);
        vec3.copy(outRight, position);
        return true;
    }
    // handle from offmesh connection to poly
    if (fromNodeType === NodeType.OFFMESH && toNodeType === NodeType.POLY) {
        const offMeshConnection = navMesh.offMeshConnections[fromNode.offMeshConnectionId];
        if (!offMeshConnection)
            return false;
        const position = toLink && toLink.edge === 0 ? offMeshConnection.start : offMeshConnection.end;
        vec3.copy(outLeft, position);
        vec3.copy(outRight, position);
        return true;
    }
    return false;
};
const _edgeMidPointPortalLeft = vec3.create();
const _edgeMidPointPortalRight = vec3.create();
const getEdgeMidPoint = (navMesh, fromNodeRef, toNodeRef, outMidPoint) => {
    if (!getPortalPoints(navMesh, fromNodeRef, toNodeRef, _edgeMidPointPortalLeft, _edgeMidPointPortalRight)) {
        return false;
    }
    outMidPoint[0] = (_edgeMidPointPortalLeft[0] + _edgeMidPointPortalRight[0]) * 0.5;
    outMidPoint[1] = (_edgeMidPointPortalLeft[1] + _edgeMidPointPortalRight[1]) * 0.5;
    outMidPoint[2] = (_edgeMidPointPortalLeft[2] + _edgeMidPointPortalRight[2]) * 0.5;
    return true;
};
var FindNodePathResultFlags;
(function (FindNodePathResultFlags) {
    FindNodePathResultFlags[FindNodePathResultFlags["NONE"] = 0] = "NONE";
    FindNodePathResultFlags[FindNodePathResultFlags["SUCCESS"] = 1] = "SUCCESS";
    FindNodePathResultFlags[FindNodePathResultFlags["COMPLETE_PATH"] = 2] = "COMPLETE_PATH";
    FindNodePathResultFlags[FindNodePathResultFlags["PARTIAL_PATH"] = 4] = "PARTIAL_PATH";
    FindNodePathResultFlags[FindNodePathResultFlags["INVALID_INPUT"] = 8] = "INVALID_INPUT";
})(FindNodePathResultFlags || (FindNodePathResultFlags = {}));
const HEURISTIC_SCALE = 0.999; // Search heuristic scale
/**
 * Find a path between two nodes.
 *
 * If the end node cannot be reached through the navigation graph,
 * the last node in the path will be the nearest the end node.
 *
 * The start and end positions are used to calculate traversal costs.
 * (The y-values impact the result.)
 *
 * @param startNodeRef The reference ID of the starting node.
 * @param endNodeRef The reference ID of the ending node.
 * @param startPosition The starting position in world space.
 * @param endPosition The ending position in world space.
 * @param filter The query filter.
 * @returns The result of the pathfinding operation.
 */
const findNodePath = (navMesh, startNodeRef, endNodeRef, startPosition, endPosition, filter) => {
    const nodes = {};
    const openList = [];
    // validate input
    if (!isValidNodeRef(navMesh, startNodeRef) ||
        !isValidNodeRef(navMesh, endNodeRef) ||
        !vec3.finite(startPosition) ||
        !vec3.finite(endPosition)) {
        return {
            flags: FindNodePathResultFlags.NONE | FindNodePathResultFlags.INVALID_INPUT,
            success: false,
            path: [],
            nodes,
            openList,
        };
    }
    // early exit if start and end are the same
    if (startNodeRef === endNodeRef) {
        return {
            flags: FindNodePathResultFlags.SUCCESS | FindNodePathResultFlags.COMPLETE_PATH,
            success: true,
            path: [startNodeRef],
            nodes,
            openList,
        };
    }
    // prepare search
    const getCost = filter.getCost;
    const startNode = {
        cost: 0,
        total: vec3.distance(startPosition, endPosition) * HEURISTIC_SCALE,
        parentNodeRef: null,
        parentState: null,
        nodeRef: startNodeRef,
        state: 0,
        flags: NODE_FLAG_OPEN,
        position: [startPosition[0], startPosition[1], startPosition[2]],
    };
    addSearchNode(nodes, startNode);
    pushNodeToQueue(openList, startNode);
    let lastBestNode = startNode;
    let lastBestNodeCost = startNode.total;
    while (openList.length > 0) {
        // remove node from the open list and put it in the closed list
        const bestSearchNode = popNodeFromQueue(openList);
        bestSearchNode.flags &= ~NODE_FLAG_OPEN;
        bestSearchNode.flags |= NODE_FLAG_CLOSED;
        // if we have reached the goal, stop searching
        const bestNodeRef = bestSearchNode.nodeRef;
        if (bestNodeRef === endNodeRef) {
            lastBestNode = bestSearchNode;
            break;
        }
        // get best node
        const bestNode = getNodeByRef(navMesh, bestNodeRef);
        // get parent node ref
        const parentNodeRef = bestSearchNode.parentNodeRef ?? undefined;
        // expand the search with node links
        for (const linkIndex of bestNode.links) {
            const link = navMesh.links[linkIndex];
            const neighbourNodeRef = link.toNodeRef;
            // do not expand back to where we came from
            if (neighbourNodeRef === parentNodeRef) {
                continue;
            }
            // check whether neighbour passes the filter
            if (filter.passFilter(neighbourNodeRef, navMesh) === false) {
                continue;
            }
            // deal explicitly with crossing tile boundaries by partitioning the search node refs by crossing side
            let state = 0;
            if (link.side !== 0xff) {
                state = link.side >> 1;
            }
            // get the neighbour node
            let neighbourSearchNode = getSearchNode(nodes, neighbourNodeRef, state);
            if (!neighbourSearchNode) {
                neighbourSearchNode = {
                    cost: 0,
                    total: 0,
                    parentNodeRef: null,
                    parentState: null,
                    nodeRef: neighbourNodeRef,
                    state,
                    flags: 0,
                    position: [0, 0, 0],
                };
                addSearchNode(nodes, neighbourSearchNode);
                getEdgeMidPoint(navMesh, bestNodeRef, neighbourNodeRef, neighbourSearchNode.position);
            }
            // calculate cost and heuristic
            let cost = 0;
            let heuristic = 0;
            // normal cost calculation
            const curCost = getCost(bestSearchNode.position, neighbourSearchNode.position, navMesh, parentNodeRef, bestNodeRef, neighbourNodeRef);
            cost = bestSearchNode.cost + curCost;
            // special case for last node - add cost to reach end position
            if (neighbourNodeRef === endNodeRef) {
                const endCost = getCost(neighbourSearchNode.position, endPosition, navMesh, bestNodeRef, neighbourNodeRef, undefined);
                cost = cost + endCost;
                heuristic = 0;
            }
            else {
                heuristic = vec3.distance(neighbourSearchNode.position, endPosition) * HEURISTIC_SCALE;
            }
            const total = cost + heuristic;
            // if the node is already in the open list, and the new result is worse, skip
            if (neighbourSearchNode.flags & NODE_FLAG_OPEN && total >= neighbourSearchNode.total) {
                continue;
            }
            // if the node is already visited and in the closed list, and the new result is worse, skip
            if (neighbourSearchNode.flags & NODE_FLAG_CLOSED && total >= neighbourSearchNode.total) {
                continue;
            }
            // add or update the node
            neighbourSearchNode.parentNodeRef = bestSearchNode.nodeRef;
            neighbourSearchNode.parentState = bestSearchNode.state;
            neighbourSearchNode.nodeRef = neighbourNodeRef;
            neighbourSearchNode.flags = neighbourSearchNode.flags & ~NODE_FLAG_CLOSED;
            neighbourSearchNode.cost = cost;
            neighbourSearchNode.total = total;
            if (neighbourSearchNode.flags & NODE_FLAG_OPEN) {
                // already in open list, update node location
                reindexNodeInQueue(openList, neighbourSearchNode);
            }
            else {
                // put the node in the open list
                neighbourSearchNode.flags |= NODE_FLAG_OPEN;
                pushNodeToQueue(openList, neighbourSearchNode);
            }
            // update nearest node to target so far
            if (heuristic < lastBestNodeCost) {
                lastBestNode = neighbourSearchNode;
                lastBestNodeCost = heuristic;
            }
        }
    }
    // assemble the path to the node
    const path = [];
    let currentNode = lastBestNode;
    while (currentNode) {
        path.push(currentNode.nodeRef);
        if (currentNode.parentNodeRef !== null && currentNode.parentState !== null) {
            currentNode = getSearchNode(nodes, currentNode.parentNodeRef, currentNode.parentState) ?? null;
        }
        else {
            currentNode = null;
        }
    }
    path.reverse();
    // if the end node was not reached, return with the partial result status
    if (lastBestNode.nodeRef !== endNodeRef) {
        return {
            flags: FindNodePathResultFlags.PARTIAL_PATH,
            success: true,
            path,
            nodes,
            openList,
        };
    }
    // the path is complete, return with the complete path status
    return {
        flags: FindNodePathResultFlags.SUCCESS | FindNodePathResultFlags.COMPLETE_PATH,
        success: true,
        path,
        nodes,
        openList,
    };
};
var SlicedFindNodePathStatusFlags;
(function (SlicedFindNodePathStatusFlags) {
    SlicedFindNodePathStatusFlags[SlicedFindNodePathStatusFlags["NOT_INITIALIZED"] = 0] = "NOT_INITIALIZED";
    SlicedFindNodePathStatusFlags[SlicedFindNodePathStatusFlags["IN_PROGRESS"] = 1] = "IN_PROGRESS";
    SlicedFindNodePathStatusFlags[SlicedFindNodePathStatusFlags["SUCCESS"] = 2] = "SUCCESS";
    SlicedFindNodePathStatusFlags[SlicedFindNodePathStatusFlags["PARTIAL_RESULT"] = 4] = "PARTIAL_RESULT";
    SlicedFindNodePathStatusFlags[SlicedFindNodePathStatusFlags["FAILURE"] = 8] = "FAILURE";
    SlicedFindNodePathStatusFlags[SlicedFindNodePathStatusFlags["INVALID_PARAM"] = 16] = "INVALID_PARAM";
})(SlicedFindNodePathStatusFlags || (SlicedFindNodePathStatusFlags = {}));
var SlicedFindNodePathInitFlags;
(function (SlicedFindNodePathInitFlags) {
    /** Enable any-angle pathfinding with raycast optimization */
    SlicedFindNodePathInitFlags[SlicedFindNodePathInitFlags["ANY_ANGLE"] = 1] = "ANY_ANGLE";
})(SlicedFindNodePathInitFlags || (SlicedFindNodePathInitFlags = {}));
/**
 * Creates a new sliced path query object with default values.
 * @returns A new sliced path query ready for initialization
 */
const createSlicedNodePathQuery = () => ({
    status: SlicedFindNodePathStatusFlags.NOT_INITIALIZED,
    startNodeRef: 0,
    endNodeRef: 0,
    startPosition: [0, 0, 0],
    endPosition: [0, 0, 0],
    filter: DEFAULT_QUERY_FILTER,
    nodes: {},
    openList: [],
    lastBestNode: null,
    lastBestNodeCost: Infinity,
    raycastLimitSqr: null,
});
/**
 * Initializes a sliced path query.
 * @param navMesh The navigation mesh
 * @param query The sliced path query to initialize
 * @param startNodeRef The reference ID of the starting node
 * @param endNodeRef The reference ID of the ending node
 * @param startPosition The starting position in world space
 * @param endPosition The ending position in world space
 * @param filter The query filter
 * @param flags Optional flags for the query (@see SlicedFindNodePathInitFlags)
 * @returns The status of the initialization
 */
const initSlicedFindNodePath = (navMesh, query, startNodeRef, endNodeRef, startPosition, endPosition, filter, flags = 0) => {
    // set search parameters
    query.startNodeRef = startNodeRef;
    query.endNodeRef = endNodeRef;
    vec3.copy(query.startPosition, startPosition);
    vec3.copy(query.endPosition, endPosition);
    query.filter = filter;
    // reset search state
    query.status = SlicedFindNodePathStatusFlags.FAILURE;
    query.nodes = {};
    query.openList = [];
    query.lastBestNode = null;
    query.lastBestNodeCost = Infinity;
    // validate input
    if (!isValidNodeRef(navMesh, startNodeRef) ||
        !isValidNodeRef(navMesh, endNodeRef) ||
        !vec3.finite(startPosition) ||
        !vec3.finite(endPosition)) {
        query.status = SlicedFindNodePathStatusFlags.FAILURE | SlicedFindNodePathStatusFlags.INVALID_PARAM;
        return query.status;
    }
    // Handle raycast optimization
    if (flags & SlicedFindNodePathInitFlags.ANY_ANGLE) {
        // Set raycast limit for any-angle pathfinding
        query.raycastLimitSqr = 25.0; // Reasonable default value
        // TODO: limiting to several times the character radius yields nice results. It is not sensitive
        // so it is enough to compute it from the first tile.
        // const dtMeshTile* tile = m_nav->getTileByRef(startRef);
        // float agentRadius = tile->header->walkableRadius;
        // m_query.raycastLimitSqr = dtSqr(agentRadius * DT_RAY_CAST_LIMIT_PROPORTIONS);
    }
    else {
        // Clear raycast optimization
        query.raycastLimitSqr = null;
    }
    // start node
    const startNode = {
        cost: 0,
        total: vec3.distance(startPosition, endPosition) * HEURISTIC_SCALE,
        parentNodeRef: null,
        parentState: null,
        nodeRef: startNodeRef,
        state: 0,
        flags: NODE_FLAG_OPEN,
        position: [startPosition[0], startPosition[1], startPosition[2]],
    };
    addSearchNode(query.nodes, startNode);
    query.lastBestNode = startNode;
    query.lastBestNodeCost = startNode.total;
    // early exit if the start poly is the end poly
    if (startNodeRef === endNodeRef) {
        query.status = SlicedFindNodePathStatusFlags.SUCCESS;
        return query.status;
    }
    pushNodeToQueue(query.openList, startNode);
    query.status = SlicedFindNodePathStatusFlags.IN_PROGRESS;
    return query.status;
};
/**
 * Updates an in-progress sliced path query.
 *
 * @param navMesh The navigation mesh
 * @param query The sliced path query to update
 * @param maxIterations The maximum number of iterations to perform
 * @returns iterations performed
 */
const updateSlicedFindNodePath = (navMesh, query, maxIterations) => {
    let itersDone = 0;
    // check if query is in valid state
    if (!(query.status & SlicedFindNodePathStatusFlags.IN_PROGRESS)) {
        return itersDone;
    }
    // validate refs are still valid
    if (!isValidNodeRef(navMesh, query.startNodeRef) || !isValidNodeRef(navMesh, query.endNodeRef)) {
        query.status = SlicedFindNodePathStatusFlags.FAILURE;
        return itersDone;
    }
    const getCost = query.filter.getCost;
    while (itersDone < maxIterations && query.openList.length > 0) {
        itersDone++;
        // remove best node from open list and close it
        const bestSearchNode = popNodeFromQueue(query.openList);
        bestSearchNode.flags &= ~NODE_FLAG_OPEN;
        bestSearchNode.flags |= NODE_FLAG_CLOSED;
        // check if we've reached the goal
        if (bestSearchNode.nodeRef === query.endNodeRef) {
            query.lastBestNode = bestSearchNode;
            query.status = SlicedFindNodePathStatusFlags.SUCCESS;
            return itersDone;
        }
        // get best node
        const bestNodeRef = bestSearchNode.nodeRef;
        const bestNode = getNodeByRef(navMesh, bestNodeRef);
        // get parent for backtracking prevention
        const parentNodeRef = bestSearchNode.parentNodeRef ?? undefined;
        // expand to neighbors
        for (const linkIndex of bestNode.links) {
            const link = navMesh.links[linkIndex];
            const neighbourNodeRef = link.toNodeRef;
            // skip parent nodes
            if (neighbourNodeRef === parentNodeRef) {
                continue;
            }
            // apply filter
            if (!query.filter.passFilter(neighbourNodeRef, navMesh)) {
                continue;
            }
            // handle tile boundary crossing
            let state = 0;
            if (link.side !== 0xff) {
                state = link.side >> 1;
            }
            // get or create neighbor node
            let neighbourSearchNode = getSearchNode(query.nodes, neighbourNodeRef, state);
            if (!neighbourSearchNode) {
                neighbourSearchNode = {
                    cost: 0,
                    total: 0,
                    parentNodeRef: null,
                    parentState: null,
                    nodeRef: neighbourNodeRef,
                    state,
                    flags: 0,
                    position: [0, 0, 0],
                };
                addSearchNode(query.nodes, neighbourSearchNode);
                getEdgeMidPoint(navMesh, bestNodeRef, neighbourNodeRef, neighbourSearchNode.position);
            }
            // calculate costs
            let cost = 0;
            let heuristic = 0;
            // check for raycast shortcut (if enabled)
            let foundShortcut = false;
            if (query.raycastLimitSqr && bestSearchNode.parentNodeRef !== null && bestSearchNode.parentState !== null) {
                // get grandparent node for potential raycast shortcut
                const grandparentNode = getSearchNode(query.nodes, bestSearchNode.parentNodeRef, bestSearchNode.parentState);
                if (grandparentNode) {
                    const rayLength = vec3.distance(grandparentNode.position, neighbourSearchNode.position);
                    if (rayLength < Math.sqrt(query.raycastLimitSqr)) {
                        // attempt raycast from grandparent to current neighbor
                        const rayResult = raycastWithCosts(navMesh, grandparentNode.nodeRef, grandparentNode.position, neighbourSearchNode.position, query.filter, grandparentNode.parentNodeRef ?? 0);
                        // if the raycast didn't hit anything, we can take the shortcut
                        if (rayResult.t >= 1.0) {
                            foundShortcut = true;
                            cost = grandparentNode.cost + rayResult.pathCost;
                        }
                    }
                }
            }
            // normal cost calculation (if no shortcut found)
            if (!foundShortcut) {
                const curCost = getCost(bestSearchNode.position, neighbourSearchNode.position, navMesh, parentNodeRef, bestNodeRef, neighbourNodeRef);
                cost = bestSearchNode.cost + curCost;
            }
            // special case for last node - add cost to reach end position
            if (neighbourNodeRef === query.endNodeRef) {
                const endCost = getCost(neighbourSearchNode.position, query.endPosition, navMesh, bestNodeRef, neighbourNodeRef, undefined);
                cost = cost + endCost;
                heuristic = 0;
            }
            else {
                heuristic = vec3.distance(neighbourSearchNode.position, query.endPosition) * HEURISTIC_SCALE;
            }
            const total = cost + heuristic;
            // skip if worse than existing
            if ((neighbourSearchNode.flags & NODE_FLAG_OPEN && total >= neighbourSearchNode.total) ||
                (neighbourSearchNode.flags & NODE_FLAG_CLOSED && total >= neighbourSearchNode.total)) {
                continue;
            }
            // update node
            if (foundShortcut) {
                neighbourSearchNode.parentNodeRef = bestSearchNode.parentNodeRef;
                neighbourSearchNode.parentState = bestSearchNode.parentState;
            }
            else {
                neighbourSearchNode.parentNodeRef = bestSearchNode.nodeRef;
                neighbourSearchNode.parentState = bestSearchNode.state;
            }
            neighbourSearchNode.cost = cost;
            neighbourSearchNode.total = total;
            neighbourSearchNode.flags &= ~NODE_FLAG_CLOSED;
            // mark as detached parent if raycast shortcut was used
            if (foundShortcut) {
                neighbourSearchNode.flags |= NODE_FLAG_PARENT_DETACHED;
            }
            else {
                neighbourSearchNode.flags &= ~NODE_FLAG_PARENT_DETACHED;
            }
            if (neighbourSearchNode.flags & NODE_FLAG_OPEN) {
                reindexNodeInQueue(query.openList, neighbourSearchNode);
            }
            else {
                neighbourSearchNode.flags |= NODE_FLAG_OPEN;
                pushNodeToQueue(query.openList, neighbourSearchNode);
            }
            // update best node tracking
            if (heuristic < query.lastBestNodeCost) {
                query.lastBestNodeCost = heuristic;
                query.lastBestNode = neighbourSearchNode;
            }
        }
    }
    // check if the search is exhausted
    if (query.openList.length === 0) {
        query.status = SlicedFindNodePathStatusFlags.SUCCESS | SlicedFindNodePathStatusFlags.PARTIAL_RESULT;
    }
    return itersDone;
};
/**
 * Finalizes and returns the results of a sliced path query.
 *
 * @param navMesh The navigation mesh
 * @param query The sliced path query to finalize
 * @returns Object containing the status, path, and path count
 */
const finalizeSlicedFindNodePath = (navMesh, query) => {
    const result = {
        status: SlicedFindNodePathStatusFlags.FAILURE,
        path: [],
        pathCount: 0,
    };
    if (!query.lastBestNode) {
        query.status = SlicedFindNodePathStatusFlags.FAILURE;
        return { ...result, status: query.status };
    }
    // handle same start/end case
    if (query.startNodeRef === query.endNodeRef) {
        result.path.push(query.startNodeRef);
        result.pathCount = 1;
        result.status = SlicedFindNodePathStatusFlags.SUCCESS;
        // reset query
        query.status = SlicedFindNodePathStatusFlags.NOT_INITIALIZED;
        return result;
    }
    // check for partial result
    if (query.lastBestNode.nodeRef !== query.endNodeRef) {
        query.status |= SlicedFindNodePathStatusFlags.PARTIAL_RESULT;
    }
    const reversedPath = [];
    let currentNode = query.lastBestNode;
    while (currentNode) {
        const wasDetached = (currentNode.flags & NODE_FLAG_PARENT_DETACHED) !== 0;
        reversedPath.push({ node: currentNode, wasDetached });
        if (currentNode.parentNodeRef !== null && currentNode.parentState !== null) {
            currentNode = getSearchNode(query.nodes, currentNode.parentNodeRef, currentNode.parentState) ?? null;
        }
        else {
            currentNode = null;
        }
    }
    // reverse to get forward order
    reversedPath.reverse();
    // build final path, filling in raycast shortcuts if any-angle pathfinding was enabled
    const finalPath = [];
    if (query.raycastLimitSqr !== null) {
        // any-angle pathfinding was enabled, need to fill in raycast shortcuts
        for (let i = 0; i < reversedPath.length; i++) {
            const current = reversedPath[i];
            const next = i + 1 < reversedPath.length ? reversedPath[i + 1] : null;
            // if this node has a detached parent, we need to raycast to fill the gap
            if (next && current.wasDetached) {
                // raycast from current position to next position to get intermediate polygons
                const rayResult = raycast(navMesh, current.node.nodeRef, current.node.position, next.node.position, query.filter);
                // add all polygons from the raycast
                for (const polyRef of rayResult.path) {
                    finalPath.push(polyRef);
                }
                // the raycast ends on a poly boundary and might include the next poly
                // remove duplicate if the last poly in raycast matches the next node
                if (finalPath.length > 0 && finalPath[finalPath.length - 1] === next.node.nodeRef) {
                    finalPath.pop();
                }
            }
            else {
                // normal adjacent connection, just add the node
                finalPath.push(current.node.nodeRef);
            }
        }
    }
    else {
        // no any-angle pathfinding, just extract the node refs
        for (let i = 0; i < reversedPath.length; i++) {
            finalPath.push(reversedPath[i].node.nodeRef);
        }
    }
    result.path = finalPath;
    result.pathCount = result.path.length;
    result.status = SlicedFindNodePathStatusFlags.SUCCESS | (query.status & SlicedFindNodePathStatusFlags.PARTIAL_RESULT);
    // reset query
    query.status = SlicedFindNodePathStatusFlags.NOT_INITIALIZED;
    return result;
};
/**
 * Finalizes and returns the results of an incomplete sliced path query,
 * returning the path to the furthest polygon on the existing path that was visited during the search.
 *
 * @param navMesh The navigation mesh
 * @param query The sliced path query to finalize
 * @param existingPath An array of polygon references for the existing path
 * @returns Object containing the status, path, and path count
 */
const finalizeSlicedFindNodePathPartial = (navMesh, query, existingPath) => {
    const result = {
        status: SlicedFindNodePathStatusFlags.FAILURE,
        path: [],
        pathCount: 0,
    };
    // find furthest visited node from existing path
    let furthestNode = null;
    for (let i = existingPath.length - 1; i >= 0; i--) {
        const targetNodeRef = existingPath[i];
        // search through all nodes to find one with matching nodeRef (regardless of state)
        const nodes = query.nodes[targetNodeRef];
        if (nodes) {
            for (let j = 0; j < nodes.length; j++) {
                const node = nodes[j];
                if (node.nodeRef === targetNodeRef) {
                    furthestNode = node;
                    break;
                }
            }
        }
        if (furthestNode) {
            break;
        }
    }
    if (!furthestNode) {
        furthestNode = query.lastBestNode;
        query.status |= SlicedFindNodePathStatusFlags.PARTIAL_RESULT;
    }
    if (!furthestNode) {
        query.status = SlicedFindNodePathStatusFlags.FAILURE;
        return { ...result, status: query.status };
    }
    // handle same start/end case
    if (query.startNodeRef === query.endNodeRef) {
        result.path.push(query.startNodeRef);
        result.pathCount = 1;
        result.status = SlicedFindNodePathStatusFlags.SUCCESS;
        // reset query
        query.status = SlicedFindNodePathStatusFlags.NOT_INITIALIZED;
        return result;
    }
    // mark as partial result since we're working with an incomplete search
    query.status |= SlicedFindNodePathStatusFlags.PARTIAL_RESULT;
    const reversedPath = [];
    let currentNode = furthestNode;
    while (currentNode) {
        const wasDetached = (currentNode.flags & NODE_FLAG_PARENT_DETACHED) !== 0;
        reversedPath.push({ node: currentNode, wasDetached });
        if (currentNode.parentNodeRef !== null && currentNode.parentState !== null) {
            currentNode = getSearchNode(query.nodes, currentNode.parentNodeRef, currentNode.parentState) ?? null;
        }
        else {
            currentNode = null;
        }
    }
    // reverse to get forward order
    reversedPath.reverse();
    // build final path, filling in raycast shortcuts if any-angle pathfinding was enabled
    const finalPath = [];
    if (query.raycastLimitSqr !== null) {
        // any-angle pathfinding was enabled, need to fill in raycast shortcuts
        for (let i = 0; i < reversedPath.length; i++) {
            const current = reversedPath[i];
            const next = i + 1 < reversedPath.length ? reversedPath[i + 1] : null;
            // if this node has a detached parent, we need to raycast to fill the gap
            if (next && current.wasDetached) {
                // raycast from current position to next position to get intermediate polygons
                const rayResult = raycast(navMesh, current.node.nodeRef, current.node.position, next.node.position, query.filter);
                // add all polygons from the raycast
                for (const polyRef of rayResult.path) {
                    finalPath.push(polyRef);
                }
                // the raycast ends on a poly boundary and might include the next poly
                // remove duplicate if the last poly in raycast matches the next node
                if (finalPath.length > 0 && finalPath[finalPath.length - 1] === next.node.nodeRef) {
                    finalPath.pop();
                }
            }
            else {
                // normal adjacent connection, just add the node
                finalPath.push(current.node.nodeRef);
            }
        }
    }
    else {
        // no any-angle pathfinding, just extract the node refs
        for (let i = 0; i < reversedPath.length; i++) {
            finalPath.push(reversedPath[i].node.nodeRef);
        }
    }
    result.path = finalPath;
    result.pathCount = result.path.length;
    result.status = SlicedFindNodePathStatusFlags.SUCCESS | SlicedFindNodePathStatusFlags.PARTIAL_RESULT;
    // reset query
    query.status = SlicedFindNodePathStatusFlags.NOT_INITIALIZED;
    return result;
};
const _moveAlongSurface_vertices = [];
const _moveAlongSurface_polyHeightResult = createGetPolyHeightResult();
const _moveAlongSurface_wallEdgeVj = vec3.create();
const _moveAlongSurface_wallEdgeVi = vec3.create();
const _moveAlongSurface_linkVj = vec3.create();
const _moveAlongSurface_linkVi = vec3.create();
const _moveAlongSurface_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
/**
 * Moves from start position towards end position along the navigation mesh surface.
 *
 * This method is optimized for small delta movement and a small number of
 * polygons. If used for too great a distance, the result set will form an
 * incomplete path.
 *
 * The resultPosition will equal the endPosition if the end is reached.
 * Otherwise the closest reachable position will be returned.
 *
 * The resulting position is projected onto the surface of the navigation mesh with @see getPolyHeight.
 *
 * @param navMesh The navigation mesh
 * @param startNodeRef The reference ID of the starting polygon
 * @param startPosition The starting position [(x, y, z)]
 * @param endPosition The ending position [(x, y, z)]
 * @param filter The query filter.
 * @returns Result containing status, final position, and visited polygons
 */
const moveAlongSurface = (navMesh, startNodeRef, startPosition, endPosition, filter) => {
    const result = {
        success: false,
        position: vec3.clone(startPosition),
        nodeRef: startNodeRef,
        visited: [],
    };
    if (!isValidNodeRef(navMesh, startNodeRef) || !vec3.finite(startPosition) || !vec3.finite(endPosition)) {
        return result;
    }
    result.success = true;
    const nodes = {};
    const startNode = {
        cost: 0,
        total: 0,
        parentNodeRef: null,
        parentState: null,
        nodeRef: startNodeRef,
        state: 0,
        flags: NODE_FLAG_CLOSED,
        position: [startPosition[0], startPosition[1], startPosition[2]],
    };
    addSearchNode(nodes, startNode);
    const bestPos = vec3.clone(startPosition);
    let bestDist = Infinity;
    let bestNode = startNode;
    // search constraints
    const searchPos = vec3.create();
    vec3.lerp(searchPos, startPosition, endPosition, 0.5);
    const searchRadSqr = (vec3.distance(startPosition, endPosition) / 2.0 + 0.001) ** 2;
    // breadth-first search queue (no priority needed for this algorithm)
    const queue = [startNode];
    while (queue.length > 0) {
        const curNode = queue.shift();
        // get poly and tile
        const curRef = curNode.nodeRef;
        const tileAndPoly = getTileAndPolyByRef(curRef, navMesh);
        if (!tileAndPoly.success)
            continue;
        const { tile, poly } = tileAndPoly;
        // collect vertices
        const nv = poly.vertices.length;
        const vertices = _moveAlongSurface_vertices;
        for (let i = 0; i < nv; ++i) {
            const start = poly.vertices[i] * 3;
            vertices[i * 3] = tile.vertices[start];
            vertices[i * 3 + 1] = tile.vertices[start + 1];
            vertices[i * 3 + 2] = tile.vertices[start + 2];
        }
        // if target is inside the poly, stop search
        if (pointInPoly(endPosition, vertices, nv)) {
            bestNode = curNode;
            vec3.copy(bestPos, endPosition);
            break;
        }
        // find wall edges and find nearest point inside the walls
        for (let i = 0, j = nv - 1; i < nv; j = i++) {
            // find links to neighbours
            const neis = [];
            const node = getNodeByRef(navMesh, curRef);
            for (const linkIndex of node.links) {
                const link = navMesh.links[linkIndex];
                if (!link)
                    continue;
                const neighbourRef = link.toNodeRef;
                // check if this link corresponds to edge j
                if (link.edge === j) {
                    // check filter
                    if (!filter.passFilter(neighbourRef, navMesh)) {
                        continue;
                    }
                    neis.push(neighbourRef);
                }
            }
            if (neis.length === 0) {
                // wall edge, calc distance
                const vj = vec3.fromBuffer(_moveAlongSurface_wallEdgeVj, vertices, j * 3);
                const vi = vec3.fromBuffer(_moveAlongSurface_wallEdgeVi, vertices, i * 3);
                const { distSqr, t: tSeg } = distancePtSegSqr2d(_moveAlongSurface_distancePtSegSqr2dResult, endPosition, vj, vi);
                if (distSqr < bestDist) {
                    // update nearest distance
                    vec3.lerp(bestPos, vj, vi, tSeg);
                    bestDist = distSqr;
                    bestNode = curNode;
                }
            }
            else {
                for (const neighbourRef of neis) {
                    let neighbourNode = getSearchNode(nodes, neighbourRef, 0);
                    if (!neighbourNode) {
                        neighbourNode = {
                            cost: 0,
                            total: 0,
                            parentNodeRef: null,
                            parentState: null,
                            nodeRef: neighbourRef,
                            state: 0,
                            flags: 0,
                            position: [endPosition[0], endPosition[1], endPosition[2]],
                        };
                        addSearchNode(nodes, neighbourNode);
                    }
                    // skip if already visited
                    if (neighbourNode.flags & NODE_FLAG_CLOSED)
                        continue;
                    // skip the link if it is too far from search constraint
                    const vj = vec3.fromBuffer(_moveAlongSurface_linkVj, vertices, j * 3);
                    const vi = vec3.fromBuffer(_moveAlongSurface_linkVi, vertices, i * 3);
                    const distSqr = distancePtSegSqr2d(_moveAlongSurface_distancePtSegSqr2dResult, searchPos, vj, vi).distSqr;
                    if (distSqr > searchRadSqr)
                        continue;
                    // mark as visited and add to queue
                    neighbourNode.parentNodeRef = curNode.nodeRef;
                    neighbourNode.parentState = curNode.state;
                    neighbourNode.flags |= NODE_FLAG_CLOSED;
                    queue.push(neighbourNode);
                }
            }
        }
    }
    if (bestNode) {
        let currentNode = bestNode;
        while (currentNode) {
            result.visited.push(currentNode.nodeRef);
            if (currentNode.parentNodeRef !== null) {
                currentNode = getSearchNode(nodes, currentNode.parentNodeRef, 0) ?? null;
            }
            else {
                currentNode = null;
            }
        }
        result.visited.reverse();
        vec3.copy(result.position, bestPos);
        result.nodeRef = bestNode.nodeRef;
        // fixup height with getPolyHeight
        const tileAndPoly = getTileAndPolyByRef(result.nodeRef, navMesh);
        if (tileAndPoly.success) {
            const polyHeightResult = getPolyHeight(_moveAlongSurface_polyHeightResult, tileAndPoly.tile, tileAndPoly.poly, tileAndPoly.polyIndex, result.position);
            if (polyHeightResult.success) {
                result.position[1] = polyHeightResult.height;
            }
        }
    }
    return result;
};
const _raycast_vertices = [];
const _raycast_dir = vec3.create();
const _raycast_curPos = vec3.create();
const _raycast_lastPos = vec3.create();
const _raycast_e1Vec = vec3.create();
const _raycast_e0Vec = vec3.create();
const _raycast_eDir = vec3.create();
const _raycast_diff = vec3.create();
const _raycast_hitNormal_va = vec3.create();
const _raycast_hitNormal_vb = vec3.create();
/**
 * Internal base implementation of raycast that handles both cost calculation modes.
 *
 * Casts a 'walkability' ray along the surface of the navigation mesh from
 * the start position toward the end position.
 *
 * This method is meant to be used for quick, short distance checks.
 * The raycast ignores the y-value of the end position (2D check).
 *
 * @param navMesh The navigation mesh to use for the raycast.
 * @param startNodeRef The NodeRef for the start polygon
 * @param startPosition The starting position in world space.
 * @param endPosition The ending position in world space.
 * @param filter The query filter to apply.
 * @param calculateCosts Whether to calculate and accumulate path costs across polygons.
 * @param prevRef The reference to the polygon we came from (for accurate cost calculations).
 */
const raycastBase = (navMesh, startNodeRef, startPosition, endPosition, filter, calculateCosts, prevRef) => {
    const result = {
        t: 0,
        hitNormal: vec3.create(),
        hitEdgeIndex: -1,
        path: [],
        pathCost: 0,
    };
    // validate input
    if (!isValidNodeRef(navMesh, startNodeRef) || !vec3.finite(startPosition) || !vec3.finite(endPosition) || !filter) {
        return result;
    }
    let curRef = startNodeRef;
    let prevRefTracking = prevRef;
    const intersectSegmentPoly2DResult = createIntersectSegmentPoly2DResult();
    if (calculateCosts) {
        vec3.sub(_raycast_dir, endPosition, startPosition);
        vec3.copy(_raycast_curPos, startPosition);
    }
    while (curRef) {
        // get current tile and poly
        const tileAndPolyResult = getTileAndPolyByRef(curRef, navMesh);
        if (!tileAndPolyResult.success)
            break;
        const { tile, poly } = tileAndPolyResult;
        // collect current poly vertices
        const nv = poly.vertices.length;
        const vertices = _raycast_vertices;
        for (let i = 0; i < nv; i++) {
            const start = poly.vertices[i] * 3;
            vertices[i * 3] = tile.vertices[start];
            vertices[i * 3 + 1] = tile.vertices[start + 1];
            vertices[i * 3 + 2] = tile.vertices[start + 2];
        }
        // cast ray against current polygon
        intersectSegmentPoly2D(intersectSegmentPoly2DResult, startPosition, endPosition, nv, vertices);
        if (!intersectSegmentPoly2DResult.intersects) {
            // could not hit the polygon, keep the old t and report hit
            return result;
        }
        result.hitEdgeIndex = intersectSegmentPoly2DResult.segMax;
        // keep track of furthest t so far
        if (intersectSegmentPoly2DResult.tmax > result.t) {
            result.t = intersectSegmentPoly2DResult.tmax;
        }
        // add polygon to visited
        result.path.push(curRef);
        // ray end is completely inside the polygon
        if (intersectSegmentPoly2DResult.segMax === -1) {
            result.t = Number.MAX_VALUE;
            return result;
        }
        // follow neighbors
        let nextRef;
        const curNode = getNodeByRef(navMesh, curRef);
        for (const linkIndex of curNode.links) {
            const link = navMesh.links[linkIndex];
            // find link which contains this edge
            if (link.edge !== intersectSegmentPoly2DResult.segMax)
                continue;
            // skip off-mesh connections
            if (getNodeRefType(link.toNodeRef) === NodeType.OFFMESH)
                continue;
            // get pointer to the next polygon
            const nextTileAndPolyResult = getTileAndPolyByRef(link.toNodeRef, navMesh);
            if (!nextTileAndPolyResult.success)
                continue;
            // skip links based on filter
            if (!filter.passFilter(link.toNodeRef, navMesh))
                continue;
            // if the link is internal, just return the ref
            if (link.side === 0xff) {
                nextRef = link.toNodeRef;
                break;
            }
            // if the link is at tile boundary, check if the link spans the whole edge
            if (link.bmin === 0 && link.bmax === 255) {
                nextRef = link.toNodeRef;
                break;
            }
            // check for partial edge links
            const v0 = poly.vertices[link.edge];
            const v1 = poly.vertices[(link.edge + 1) % poly.vertices.length];
            const left = [tile.vertices[v0 * 3], tile.vertices[v0 * 3 + 1], tile.vertices[v0 * 3 + 2]];
            const right = [tile.vertices[v1 * 3], tile.vertices[v1 * 3 + 1], tile.vertices[v1 * 3 + 2]];
            // check that the intersection lies inside the link portal
            if (link.side === 0 || link.side === 4) {
                // calculate link size
                const s = 1.0 / 255.0;
                let lmin = left[2] + (right[2] - left[2]) * (link.bmin * s);
                let lmax = left[2] + (right[2] - left[2]) * (link.bmax * s);
                if (lmin > lmax)
                    [lmin, lmax] = [lmax, lmin];
                // find Z intersection
                const z = startPosition[2] + (endPosition[2] - startPosition[2]) * intersectSegmentPoly2DResult.tmax;
                if (z >= lmin && z <= lmax) {
                    nextRef = link.toNodeRef;
                    break;
                }
            }
            else if (link.side === 2 || link.side === 6) {
                // calculate link size
                const s = 1.0 / 255.0;
                let lmin = left[0] + (right[0] - left[0]) * (link.bmin * s);
                let lmax = left[0] + (right[0] - left[0]) * (link.bmax * s);
                if (lmin > lmax)
                    [lmin, lmax] = [lmax, lmin];
                // find X intersection
                const x = startPosition[0] + (endPosition[0] - startPosition[0]) * intersectSegmentPoly2DResult.tmax;
                if (x >= lmin && x <= lmax) {
                    nextRef = link.toNodeRef;
                    break;
                }
            }
        }
        if (nextRef === undefined) {
            // no neighbor, we hit a wall
            // calculate hit normal
            if (intersectSegmentPoly2DResult.segMax >= 0) {
                const a = intersectSegmentPoly2DResult.segMax;
                const b = intersectSegmentPoly2DResult.segMax + 1 < poly.vertices.length ? intersectSegmentPoly2DResult.segMax + 1 : 0;
                vec3.fromBuffer(_raycast_hitNormal_va, vertices, a * 3);
                vec3.fromBuffer(_raycast_hitNormal_vb, vertices, b * 3);
                const dx = _raycast_hitNormal_vb[0] - _raycast_hitNormal_va[0];
                const dz = _raycast_hitNormal_vb[2] - _raycast_hitNormal_va[2];
                result.hitNormal[0] = dz;
                result.hitNormal[1] = 0;
                result.hitNormal[2] = -dx;
                vec3.normalize(result.hitNormal, result.hitNormal);
            }
            return result;
        }
        // calculate cost along the path
        if (calculateCosts) {
            // update curPos to the hit point on this polygon's edge
            vec3.copy(_raycast_lastPos, _raycast_curPos);
            vec3.scaleAndAdd(_raycast_curPos, startPosition, _raycast_dir, result.t);
            // calculate height at the hit point by lerping vertices of the hit edge
            const e0 = poly.vertices[intersectSegmentPoly2DResult.segMax];
            const e1 = poly.vertices[(intersectSegmentPoly2DResult.segMax + 1) % poly.vertices.length];
            const e1Offset = e1 * 3;
            const e0Offset = e0 * 3;
            vec3.fromBuffer(_raycast_e1Vec, tile.vertices, e1Offset);
            vec3.fromBuffer(_raycast_e0Vec, tile.vertices, e0Offset);
            vec3.sub(_raycast_eDir, _raycast_e1Vec, _raycast_e0Vec);
            vec3.sub(_raycast_diff, _raycast_curPos, _raycast_e0Vec);
            // use squared comparison to determine which component to use
            const s = _raycast_eDir[0] * _raycast_eDir[0] > _raycast_eDir[2] * _raycast_eDir[2]
                ? _raycast_diff[0] / _raycast_eDir[0]
                : _raycast_diff[2] / _raycast_eDir[2];
            _raycast_curPos[1] = _raycast_e0Vec[1] + _raycast_eDir[1] * s;
            // accumulate cost
            result.pathCost += filter.getCost(_raycast_lastPos, _raycast_curPos, navMesh, prevRefTracking, curRef, nextRef);
        }
        // no hit, advance to neighbor polygon
        prevRefTracking = curRef;
        curRef = nextRef;
    }
    return result;
};
/**
 * Casts a 'walkability' ray along the surface of the navigation mesh from
 * the start position toward the end position.
 *
 * This method is meant to be used for quick, short distance checks.
 * The raycast ignores the y-value of the end position (2D check).
 *
 * @param navMesh The navigation mesh to use for the raycast.
 * @param startNodeRef The NodeRef for the start polygon
 * @param startPosition The starting position in world space.
 * @param endPosition The ending position in world space.
 * @param filter The query filter to apply.
 * @returns The raycast result with hit information and visited polygons (without cost calculation).
 */
const raycast = (navMesh, startNodeRef, startPosition, endPosition, filter) => {
    return raycastBase(navMesh, startNodeRef, startPosition, endPosition, filter, false, 0);
};
/**
 * Casts a 'walkability' ray along the surface of the navigation mesh from
 * the start position toward the end position, calculating accumulated path costs.
 *
 * This method is meant to be used for quick, short distance checks.
 * The raycast ignores the y-value of the end position (2D check).
 *
 * @param navMesh The navigation mesh to use for the raycast.
 * @param startNodeRef The NodeRef for the start polygon
 * @param startPosition The starting position in world space.
 * @param endPosition The ending position in world space.
 * @param filter The query filter to apply.
 * @param prevRef The reference to the polygon we came from (for accurate cost calculations).
 * @returns The raycast result with hit information, visited polygons, and accumulated path costs.
 */
const raycastWithCosts = (navMesh, startNodeRef, startPosition, endPosition, filter, prevRef) => {
    return raycastBase(navMesh, startNodeRef, startPosition, endPosition, filter, true, prevRef);
};
const _findRandomPointVertices = [];
/**
 * Finds a random point on the navigation mesh.
 *
 * @param navMesh The navigation mesh
 * @param filter Query filter to apply to polygons
 * @param rand Function that returns random values between [0,1]
 * @returns The result object with success flag, random point, and polygon reference
 */
const findRandomPoint = (navMesh, filter, rand) => {
    const result = {
        success: false,
        nodeRef: 0,
        position: [0, 0, 0],
    };
    // randomly pick one tile using reservoir sampling
    let selectedTile = null;
    let tileSum = 0;
    const tiles = Object.values(navMesh.tiles);
    for (const tile of tiles) {
        if (!tile || !tile.polys)
            continue;
        // choose random tile using reservoir sampling
        const area = 1.0; // could be tile area, but we use uniform weighting
        tileSum += area;
        const u = rand();
        if (u * tileSum <= area) {
            selectedTile = tile;
        }
    }
    if (!selectedTile) {
        return result;
    }
    // randomly pick one polygon weighted by polygon area
    let selectedPoly = null;
    let selectedPolyRef = null;
    let areaSum = 0;
    for (let i = 0; i < selectedTile.polys.length; i++) {
        const poly = selectedTile.polys[i];
        const node = getNodeByTileAndPoly(navMesh, selectedTile, i);
        // must pass filter
        if (!filter.passFilter(node.ref, navMesh)) {
            continue;
        }
        // calculate area of the polygon using triangulation
        let polyArea = 0;
        const va = vec3.create();
        const vb = vec3.create();
        const vc = vec3.create();
        for (let j = 2; j < poly.vertices.length; j++) {
            vec3.fromBuffer(va, selectedTile.vertices, poly.vertices[0] * 3);
            vec3.fromBuffer(vb, selectedTile.vertices, poly.vertices[j - 1] * 3);
            vec3.fromBuffer(vc, selectedTile.vertices, poly.vertices[j] * 3);
            polyArea += triArea2D(va, vb, vc);
        }
        // choose random polygon weighted by area, using reservoir sampling
        areaSum += polyArea;
        const u = rand();
        if (u * areaSum <= polyArea) {
            selectedPoly = poly;
            selectedPolyRef = node.ref;
        }
    }
    if (!selectedPoly || !selectedPolyRef) {
        return result;
    }
    // randomly pick point on polygon
    const nv = selectedPoly.vertices.length;
    const vertices = _findRandomPointVertices;
    for (let j = 0; j < nv; j++) {
        const start = selectedPoly.vertices[j] * 3;
        vertices[j * 3] = selectedTile.vertices[start];
        vertices[j * 3 + 1] = selectedTile.vertices[start + 1];
        vertices[j * 3 + 2] = selectedTile.vertices[start + 2];
    }
    const s = rand();
    const t = rand();
    const areas = new Array(nv);
    const pt = [0, 0, 0];
    randomPointInConvexPoly(pt, nv, vertices, areas, s, t);
    // project point onto polygon surface to ensure it's exactly on the mesh
    const closestPointResult = createGetClosestPointOnPolyResult();
    getClosestPointOnPoly(closestPointResult, navMesh, selectedPolyRef, pt);
    if (closestPointResult.success) {
        vec3.copy(result.position, closestPointResult.position);
    }
    else {
        vec3.copy(result.position, pt);
    }
    result.nodeRef = selectedPolyRef;
    result.success = true;
    return result;
};
const _findRandomPointAroundCircleVertices = [];
const _findRandomPointAroundCircle_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
/**
 * Finds a random point within a circle around a center position on the navigation mesh.
 *
 * Uses Dijkstra-like search to explore reachable polygons within the circle,
 * then selects a random polygon weighted by area, and finally generates
 * a random point within that polygon.
 *
 * @param navMesh The navigation mesh
 * @param startNodeRef Reference to the polygon to start the search from
 * @param position Center position of the search circle
 * @param maxRadius Maximum radius of the search circle
 * @param filter Query filter to apply to polygons
 * @param rand Function that returns random values [0,1]
 * @returns The result object with success flag, random point, and polygon reference
 */
const findRandomPointAroundCircle = (navMesh, startNodeRef, position, maxRadius, filter, rand) => {
    const result = {
        success: false,
        nodeRef: 0,
        position: [0, 0, 0],
    };
    // validate input
    if (!isValidNodeRef(navMesh, startNodeRef) || !vec3.finite(position) || maxRadius < 0 || !Number.isFinite(maxRadius)) {
        return result;
    }
    const startTileAndPoly = getTileAndPolyByRef(startNodeRef, navMesh);
    if (!startTileAndPoly.success) {
        return result;
    }
    // check if start polygon passes filter
    if (!filter.passFilter(startNodeRef, navMesh)) {
        return result;
    }
    // prepare search
    const nodes = {};
    const openList = [];
    const startNode = {
        cost: 0,
        total: 0,
        parentNodeRef: null,
        parentState: null,
        nodeRef: startNodeRef,
        state: 0,
        flags: NODE_FLAG_OPEN,
        position: [position[0], position[1], position[2]],
    };
    addSearchNode(nodes, startNode);
    pushNodeToQueue(openList, startNode);
    const radiusSqr = maxRadius * maxRadius;
    let areaSum = 0;
    let randomTile = null;
    let randomPoly = null;
    let randomPolyRef = null;
    const va = vec3.create();
    const vb = vec3.create();
    while (openList.length > 0) {
        // remove node from the open list and put it in the closed list
        const bestNode = popNodeFromQueue(openList);
        bestNode.flags &= ~NODE_FLAG_OPEN;
        bestNode.flags |= NODE_FLAG_CLOSED;
        // get poly and tile
        const bestRef = bestNode.nodeRef;
        const bestTileAndPoly = getTileAndPolyByRef(bestRef, navMesh);
        if (!bestTileAndPoly.success)
            continue;
        const { tile: bestTile, poly: bestPoly } = bestTileAndPoly;
        // place random locations on ground polygons
        // calculate area of the polygon
        let polyArea = 0;
        const v0 = vec3.create();
        const v1 = vec3.create();
        const v2 = vec3.create();
        for (let j = 2; j < bestPoly.vertices.length; j++) {
            vec3.fromBuffer(v0, bestTile.vertices, bestPoly.vertices[0] * 3);
            vec3.fromBuffer(v1, bestTile.vertices, bestPoly.vertices[j - 1] * 3);
            vec3.fromBuffer(v2, bestTile.vertices, bestPoly.vertices[j] * 3);
            polyArea += triArea2D(v0, v1, v2);
        }
        // choose random polygon weighted by area, using reservoir sampling
        areaSum += polyArea;
        const u = rand();
        if (u * areaSum <= polyArea) {
            randomTile = bestTile;
            randomPoly = bestPoly;
            randomPolyRef = bestRef;
        }
        // get parent reference for preventing backtracking
        const parentRef = bestNode.parentNodeRef;
        // iterate through all links from the current polygon
        const node = getNodeByRef(navMesh, bestRef);
        for (const linkIndex of node.links) {
            const link = navMesh.links[linkIndex];
            if (!link)
                continue;
            const neighbourRef = link.toNodeRef;
            // skip invalid neighbours and do not follow back to parent
            if (!neighbourRef || neighbourRef === parentRef) {
                continue;
            }
            // expand to neighbour
            const neighbourTileAndPoly = getTileAndPolyByRef(neighbourRef, navMesh);
            if (!neighbourTileAndPoly.success)
                continue;
            // do not advance if the polygon is excluded by the filter
            if (!filter.passFilter(neighbourRef, navMesh)) {
                continue;
            }
            // find edge and calc distance to the edge
            if (!getPortalPoints(navMesh, bestRef, neighbourRef, va, vb)) {
                continue;
            }
            // if the circle is not touching the next polygon, skip it
            const { distSqr } = distancePtSegSqr2d(_findRandomPointAroundCircle_distancePtSegSqr2dResult, position, va, vb);
            if (distSqr > radiusSqr) {
                continue;
            }
            // get or create neighbour node
            let neighbourNode = getSearchNode(nodes, neighbourRef, 0);
            if (!neighbourNode) {
                neighbourNode = {
                    cost: 0,
                    total: 0,
                    parentNodeRef: null,
                    parentState: null,
                    nodeRef: neighbourRef,
                    state: 0,
                    flags: 0,
                    position: [0, 0, 0],
                };
                addSearchNode(nodes, neighbourNode);
            }
            if (neighbourNode.flags & NODE_FLAG_CLOSED) {
                continue;
            }
            // set position if this is the first time we visit this node
            if (neighbourNode.flags === 0) {
                vec3.lerp(neighbourNode.position, va, vb, 0.5);
            }
            const total = bestNode.total + vec3.distance(bestNode.position, neighbourNode.position);
            // the node is already in open list and the new result is worse, skip
            if (neighbourNode.flags & NODE_FLAG_OPEN && total >= neighbourNode.total) {
                continue;
            }
            neighbourNode.parentNodeRef = bestRef;
            neighbourNode.parentState = 0;
            neighbourNode.flags = neighbourNode.flags & ~NODE_FLAG_CLOSED;
            neighbourNode.total = total;
            if (neighbourNode.flags & NODE_FLAG_OPEN) {
                reindexNodeInQueue(openList, neighbourNode);
            }
            else {
                neighbourNode.flags = NODE_FLAG_OPEN;
                pushNodeToQueue(openList, neighbourNode);
            }
        }
    }
    if (!randomPoly || !randomTile || !randomPolyRef) {
        return result;
    }
    // randomly pick point on polygon
    const nv = randomPoly.vertices.length;
    const vertices = _findRandomPointAroundCircleVertices;
    for (let j = 0; j < nv; j++) {
        const start = randomPoly.vertices[j] * 3;
        vertices[j * 3] = randomTile.vertices[start];
        vertices[j * 3 + 1] = randomTile.vertices[start + 1];
        vertices[j * 3 + 2] = randomTile.vertices[start + 2];
    }
    const s = rand();
    const t = rand();
    const areas = new Array(nv);
    const pt = [0, 0, 0];
    randomPointInConvexPoly(pt, nv, vertices, areas, s, t);
    // project point onto polygon surface to ensure it's exactly on the mesh
    const closestPointResult = createGetClosestPointOnPolyResult();
    getClosestPointOnPoly(closestPointResult, navMesh, randomPolyRef, pt);
    if (closestPointResult.success) {
        vec3.copy(result.position, closestPointResult.position);
    }
    else {
        vec3.copy(result.position, pt);
    }
    result.nodeRef = randomPolyRef;
    result.success = true;
    return result;
};

var FindStraightPathOptions;
(function (FindStraightPathOptions) {
    FindStraightPathOptions[FindStraightPathOptions["ALL_CROSSINGS"] = 1] = "ALL_CROSSINGS";
    FindStraightPathOptions[FindStraightPathOptions["AREA_CROSSINGS"] = 2] = "AREA_CROSSINGS";
})(FindStraightPathOptions || (FindStraightPathOptions = {}));
var StraightPathPointFlags;
(function (StraightPathPointFlags) {
    StraightPathPointFlags[StraightPathPointFlags["START"] = 0] = "START";
    StraightPathPointFlags[StraightPathPointFlags["END"] = 1] = "END";
    StraightPathPointFlags[StraightPathPointFlags["OFFMESH"] = 2] = "OFFMESH";
})(StraightPathPointFlags || (StraightPathPointFlags = {}));
var FindStraightPathResultFlags;
(function (FindStraightPathResultFlags) {
    FindStraightPathResultFlags[FindStraightPathResultFlags["NONE"] = 0] = "NONE";
    FindStraightPathResultFlags[FindStraightPathResultFlags["SUCCESS"] = 1] = "SUCCESS";
    FindStraightPathResultFlags[FindStraightPathResultFlags["PARTIAL_PATH"] = 4] = "PARTIAL_PATH";
    FindStraightPathResultFlags[FindStraightPathResultFlags["MAX_POINTS_REACHED"] = 8] = "MAX_POINTS_REACHED";
    FindStraightPathResultFlags[FindStraightPathResultFlags["INVALID_INPUT"] = 16] = "INVALID_INPUT";
})(FindStraightPathResultFlags || (FindStraightPathResultFlags = {}));
var AppendVertexStatus;
(function (AppendVertexStatus) {
    AppendVertexStatus[AppendVertexStatus["SUCCESS"] = 1] = "SUCCESS";
    AppendVertexStatus[AppendVertexStatus["MAX_POINTS_REACHED"] = 2] = "MAX_POINTS_REACHED";
    AppendVertexStatus[AppendVertexStatus["IN_PROGRESS"] = 4] = "IN_PROGRESS";
})(AppendVertexStatus || (AppendVertexStatus = {}));
const appendVertex = (position, nodeRef, flags, outPoints, nodeType, maxPoints = null) => {
    if (outPoints.length > 0 && vec3.equals(outPoints[outPoints.length - 1].position, position)) {
        const prevType = outPoints[outPoints.length - 1].type;
        // only update if both points are regular polygon nodes
        // off-mesh connections should always be distinct waypoints
        if (prevType === NodeType.POLY && nodeType === NodeType.POLY) {
            // the vertices are equal, update
            outPoints[outPoints.length - 1].nodeRef = nodeRef;
            outPoints[outPoints.length - 1].type = nodeType;
            return AppendVertexStatus.IN_PROGRESS;
        }
        // for off-mesh connections or mixed types, fall through to append a new point
    }
    // append new vertex
    outPoints.push({
        position: [position[0], position[1], position[2]],
        type: nodeType,
        nodeRef: nodeRef,
        flags,
    });
    // if there is no space to append more vertices, return
    if (maxPoints !== null && outPoints.length >= maxPoints) {
        return AppendVertexStatus.SUCCESS | AppendVertexStatus.MAX_POINTS_REACHED;
    }
    // if reached end of path, return
    if (flags & StraightPathPointFlags.END) {
        return AppendVertexStatus.SUCCESS;
    }
    // else, continue appending points
    return AppendVertexStatus.IN_PROGRESS;
};
const _intersectSegSeg2DResult = createIntersectSegSeg2DResult();
const _appendPortalsPoint = vec3.create();
const _appendPortalsLeft = vec3.create();
const _appendPortalsRight = vec3.create();
const appendPortals = (navMesh, startIdx, endIdx, endPosition, path, outPoints, options, maxPoints = null) => {
    const startPos = outPoints[outPoints.length - 1].position;
    for (let i = startIdx; i < endIdx; i++) {
        const from = path[i];
        const to = path[i + 1];
        // skip intersection if only area crossings requested and areas equal.
        if (options & FindStraightPathOptions.AREA_CROSSINGS) {
            const a = getNodeByRef(navMesh, from);
            const b = getNodeByRef(navMesh, to);
            if (a.area === b.area)
                continue;
        }
        // calculate portal
        const left = _appendPortalsLeft;
        const right = _appendPortalsRight;
        if (!getPortalPoints(navMesh, from, to, left, right)) {
            break;
        }
        // append intersection
        const intersectResult = intersectSegSeg2D(_intersectSegSeg2DResult, startPos, endPosition, left, right);
        if (!intersectResult.hit)
            continue;
        const point = vec3.lerp(_appendPortalsPoint, left, right, intersectResult.t);
        const toType = getNodeRefType(to);
        const stat = appendVertex(point, to, 0, outPoints, toType, maxPoints);
        if (stat !== AppendVertexStatus.IN_PROGRESS) {
            return stat;
        }
    }
    return AppendVertexStatus.IN_PROGRESS;
};
const _findStraightPathLeftPortalPoint = vec3.create();
const _findStraightPathRightPortalPoint = vec3.create();
const _findStraightPath_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
const makeFindStraightPathResult = (flags, path) => ({
    flags,
    success: (flags & FindStraightPathResultFlags.SUCCESS) !== 0,
    path,
});
/**
 * This method peforms what is often called 'string pulling'.
 *
 * The start position is clamped to the first polygon node in the path, and the
 * end position is clamped to the last. So the start and end positions should
 * normally be within or very near the first and last polygons respectively.
 *
 * @param navMesh The navigation mesh to use for the search.
 * @param start The start position in world space.
 * @param end The end position in world space.
 * @param pathNodeRefs The list of polygon node references that form the path, generally obtained from `findNodePath`
 * @param maxPoints The maximum number of points to return in the straight path. If null, no limit is applied.
 * @param straightPathOptions @see FindStraightPathOptions
 * @returns The straight path
 */
const findStraightPath = (navMesh, start, end, pathNodeRefs, maxPoints = null, straightPathOptions = 0) => {
    const path = [];
    if (!vec3.finite(start) || !vec3.finite(end) || pathNodeRefs.length === 0) {
        return makeFindStraightPathResult(FindStraightPathResultFlags.NONE | FindStraightPathResultFlags.INVALID_INPUT, path);
    }
    // clamp start & end to poly boundaries
    const closestStartPos = vec3.create();
    if (!getClosestPointOnPolyBoundary(closestStartPos, navMesh, pathNodeRefs[0], start))
        return makeFindStraightPathResult(FindStraightPathResultFlags.NONE | FindStraightPathResultFlags.INVALID_INPUT, path);
    const closestEndPos = vec3.create();
    if (!getClosestPointOnPolyBoundary(closestEndPos, navMesh, pathNodeRefs[pathNodeRefs.length - 1], end))
        return makeFindStraightPathResult(FindStraightPathResultFlags.NONE | FindStraightPathResultFlags.INVALID_INPUT, path);
    // add start point
    const startAppendStatus = appendVertex(closestStartPos, pathNodeRefs[0], StraightPathPointFlags.START, path, getNodeRefType(pathNodeRefs[0]), maxPoints);
    if (startAppendStatus !== AppendVertexStatus.IN_PROGRESS) {
        // if we hit max points on the first vertex, it's a degenerate case
        const maxPointsReached = (startAppendStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
        let flags = FindStraightPathResultFlags.SUCCESS | FindStraightPathResultFlags.PARTIAL_PATH;
        if (maxPointsReached)
            flags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
        return makeFindStraightPathResult(flags, path);
    }
    const portalApex = vec3.create();
    const portalLeft = vec3.create();
    const portalRight = vec3.create();
    const pathSize = pathNodeRefs.length;
    if (pathSize > 1) {
        vec3.copy(portalApex, closestStartPos);
        vec3.copy(portalLeft, portalApex);
        vec3.copy(portalRight, portalApex);
        let apexIndex = 0;
        let leftIndex = 0;
        let rightIndex = 0;
        let leftNodeRef = pathNodeRefs[0];
        let rightNodeRef = pathNodeRefs[0];
        let leftNodeType = NodeType.POLY;
        let rightNodeType = NodeType.POLY;
        for (let i = 0; i < pathSize; ++i) {
            let toType = NodeType.POLY;
            const left = _findStraightPathLeftPortalPoint;
            const right = _findStraightPathRightPortalPoint;
            if (i + 1 < pathSize) {
                const toRef = pathNodeRefs[i + 1];
                toType = getNodeRefType(toRef);
                // next portal
                if (!getPortalPoints(navMesh, pathNodeRefs[i], toRef, left, right)) {
                    // failed to get portal points, clamp end to current poly and return partial
                    const endClamp = vec3.create();
                    // this should only happen when the first polygon is invalid.
                    if (!getClosestPointOnPolyBoundary(endClamp, navMesh, pathNodeRefs[i], end))
                        return makeFindStraightPathResult(FindStraightPathResultFlags.NONE | FindStraightPathResultFlags.INVALID_INPUT, path);
                    // append portals along the current straight path segment.
                    if (straightPathOptions & (FindStraightPathOptions.AREA_CROSSINGS | FindStraightPathOptions.ALL_CROSSINGS)) {
                        // ignore status return value as we're just about to return
                        appendPortals(navMesh, apexIndex, i, endClamp, pathNodeRefs, path, straightPathOptions, maxPoints);
                    }
                    const nodeType = getNodeRefType(pathNodeRefs[i]);
                    // ignore status return value as we're just about to return
                    appendVertex(endClamp, pathNodeRefs[i], 0, path, nodeType, maxPoints);
                    return makeFindStraightPathResult(FindStraightPathResultFlags.SUCCESS | FindStraightPathResultFlags.PARTIAL_PATH, path);
                }
                if (i === 0 && toType === NodeType.POLY) {
                    // if starting really close to the portal, advance
                    const result = distancePtSegSqr2d(_findStraightPath_distancePtSegSqr2dResult, portalApex, left, right);
                    if (result.distSqr < 1e-6)
                        continue;
                }
                // handle off-mesh connections explicitly
                // off-mesh connections should not be subject to string-pulling optimization
                if (toType === NodeType.OFFMESH) {
                    // get the off-mesh connection data
                    const node = getNodeByRef(navMesh, toRef);
                    const offMeshConnection = navMesh.offMeshConnections[node.offMeshConnectionId];
                    const offMeshConnectionAttachment = navMesh.offMeshConnectionAttachments[node.offMeshConnectionId];
                    // find the link from the previous poly to the off-mesh node to determine direction
                    const prevPolyRef = pathNodeRefs[i];
                    const prevNode = getNodeByRef(navMesh, prevPolyRef);
                    let linkEdge = 0; // default to START
                    for (const linkIndex of prevNode.links) {
                        const link = navMesh.links[linkIndex];
                        if (link.toNodeRef === toRef) {
                            linkEdge = link.edge;
                            break;
                        }
                    }
                    // use the link edge to determine direction
                    // edge 0 = entering from START side, edge 1 = entering from END side
                    const enteringFromStart = linkEdge === 0;
                    // determine start and end based on which side of the connection we're entering from
                    const offMeshStart = enteringFromStart ? offMeshConnection.start : offMeshConnection.end;
                    const offMeshEnd = enteringFromStart ? offMeshConnection.end : offMeshConnection.start;
                    // get the target polygon we'll land on after the off-mesh connection
                    const toPolyRef = enteringFromStart
                        ? offMeshConnectionAttachment.endPolyNode
                        : offMeshConnectionAttachment.startPolyNode;
                    // append any pending portals along the current straight path segment
                    // this ensures we add intermediate waypoints between the current apex and the off-mesh connection start
                    const appendPortalsStatus = appendPortals(navMesh, apexIndex, i, offMeshStart, pathNodeRefs, path, straightPathOptions, maxPoints);
                    if (appendPortalsStatus !== AppendVertexStatus.IN_PROGRESS) {
                        const maxPointsReached = (appendPortalsStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                        let flags = FindStraightPathResultFlags.SUCCESS | FindStraightPathResultFlags.PARTIAL_PATH;
                        if (maxPointsReached)
                            flags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                        return makeFindStraightPathResult(flags, path);
                    }
                    // append the off-mesh connection start point (with OFFMESH flag)
                    const appendStartStatus = appendVertex(offMeshStart, toRef, StraightPathPointFlags.OFFMESH, path, NodeType.OFFMESH, maxPoints);
                    if (appendStartStatus !== AppendVertexStatus.IN_PROGRESS) {
                        const maxPointsReached = (appendStartStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                        let resultFlags = FindStraightPathResultFlags.SUCCESS;
                        if (maxPointsReached)
                            resultFlags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                        return makeFindStraightPathResult(resultFlags, path);
                    }
                    // append the off-mesh connection end point (landing point on target polygon)
                    const appendEndStatus = appendVertex(offMeshEnd, toPolyRef, 0, path, NodeType.POLY, maxPoints);
                    if (appendEndStatus !== AppendVertexStatus.IN_PROGRESS) {
                        const maxPointsReached = (appendEndStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                        let resultFlags = FindStraightPathResultFlags.SUCCESS;
                        if (maxPointsReached)
                            resultFlags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                        return makeFindStraightPathResult(resultFlags, path);
                    }
                    // reset the funnel, we should start from the ground poly at the end of the off-mesh connection
                    vec3.copy(portalApex, offMeshEnd);
                    vec3.copy(portalLeft, offMeshEnd);
                    vec3.copy(portalRight, offMeshEnd);
                    // set apex to the landing polygon (i+1) rather than the off-mesh node (i)
                    // this prevents infinite loops: if the funnel restarts via `i = apexIndex`,
                    // we want to restart from the landing polygon, not re-enter the off-mesh handler
                    apexIndex = i + 1;
                    leftIndex = i + 1;
                    rightIndex = i + 1;
                    leftNodeRef = toPolyRef;
                    rightNodeRef = toPolyRef;
                    leftNodeType = NodeType.POLY;
                    rightNodeType = NodeType.POLY;
                    // skip normal funnel processing for this off-mesh connection
                    continue;
                }
            }
            else {
                // end of path
                vec3.copy(left, closestEndPos);
                vec3.copy(right, closestEndPos);
                toType = NodeType.POLY;
            }
            // right vertex
            if (triArea2D(portalApex, portalRight, right) <= 0.0) {
                if (vec3.equals(portalApex, portalRight) || triArea2D(portalApex, portalLeft, right) > 0.0) {
                    vec3.copy(portalRight, right);
                    rightNodeRef = i + 1 < pathSize ? pathNodeRefs[i + 1] : null;
                    rightNodeType = toType;
                    rightIndex = i;
                }
                else {
                    // append portals along current straight segment
                    if (straightPathOptions & (FindStraightPathOptions.AREA_CROSSINGS | FindStraightPathOptions.ALL_CROSSINGS)) {
                        const appendStatus = appendPortals(navMesh, apexIndex, leftIndex, portalLeft, pathNodeRefs, path, straightPathOptions, maxPoints);
                        if (appendStatus !== AppendVertexStatus.IN_PROGRESS) {
                            const maxPointsReached = (appendStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                            let flags = FindStraightPathResultFlags.SUCCESS | FindStraightPathResultFlags.PARTIAL_PATH;
                            if (maxPointsReached)
                                flags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                            return makeFindStraightPathResult(flags, path);
                        }
                    }
                    vec3.copy(portalApex, portalLeft);
                    apexIndex = leftIndex;
                    let pointFlags = 0;
                    if (!leftNodeRef) {
                        pointFlags = StraightPathPointFlags.END;
                    }
                    // note: leftNodeType can never be OFFMESH here because off-mesh connections are handled explicitly above
                    // append or update vertex
                    const appendStatus = appendVertex(portalApex, leftNodeRef, pointFlags, path, leftNodeRef ? leftNodeType : NodeType.POLY, maxPoints);
                    if (appendStatus !== AppendVertexStatus.IN_PROGRESS) {
                        const maxPointsReached = (appendStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                        let resultFlags = 0;
                        resultFlags |= FindStraightPathResultFlags.SUCCESS;
                        if (maxPointsReached)
                            resultFlags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                        return makeFindStraightPathResult(resultFlags, path);
                    }
                    vec3.copy(portalLeft, portalApex);
                    vec3.copy(portalRight, portalApex);
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    // restart
                    i = apexIndex;
                    continue;
                }
            }
            // left vertex
            if (triArea2D(portalApex, portalLeft, _findStraightPathLeftPortalPoint) >= 0.0) {
                if (vec3.equals(portalApex, portalLeft) ||
                    triArea2D(portalApex, portalRight, _findStraightPathLeftPortalPoint) < 0.0) {
                    vec3.copy(portalLeft, _findStraightPathLeftPortalPoint);
                    leftNodeRef = i + 1 < pathSize ? pathNodeRefs[i + 1] : null;
                    leftNodeType = toType;
                    leftIndex = i;
                }
                else {
                    // append portals along current straight segment
                    if (straightPathOptions & (FindStraightPathOptions.AREA_CROSSINGS | FindStraightPathOptions.ALL_CROSSINGS)) {
                        const appendStatus = appendPortals(navMesh, apexIndex, rightIndex, portalRight, pathNodeRefs, path, straightPathOptions, maxPoints);
                        if (appendStatus !== AppendVertexStatus.IN_PROGRESS) {
                            const maxPointsReached = (appendStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                            let flags = FindStraightPathResultFlags.SUCCESS | FindStraightPathResultFlags.PARTIAL_PATH;
                            if (maxPointsReached)
                                flags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                            return makeFindStraightPathResult(flags, path);
                        }
                    }
                    vec3.copy(portalApex, portalRight);
                    apexIndex = rightIndex;
                    let pointFlags = 0;
                    if (!rightNodeRef) {
                        pointFlags = StraightPathPointFlags.END;
                    }
                    // note: rightNodeType can never be OFFMESH here because off-mesh connections are handled explicitly above
                    // add/update vertex
                    const appendStatus = appendVertex(portalApex, rightNodeRef, pointFlags, path, rightNodeRef ? rightNodeType : NodeType.POLY, maxPoints);
                    if (appendStatus !== AppendVertexStatus.IN_PROGRESS) {
                        const maxPointsReached = (appendStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                        let resultFlags = 0;
                        resultFlags |= FindStraightPathResultFlags.SUCCESS;
                        if (maxPointsReached)
                            resultFlags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                        return makeFindStraightPathResult(resultFlags, path);
                    }
                    vec3.copy(portalLeft, portalApex);
                    vec3.copy(portalRight, portalApex);
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    // restart
                    i = apexIndex;
                    continue;
                }
            }
        }
        // append portals along the current straight path segment
        if (straightPathOptions & (FindStraightPathOptions.AREA_CROSSINGS | FindStraightPathOptions.ALL_CROSSINGS)) {
            const appendStatus = appendPortals(navMesh, apexIndex, pathSize - 1, closestEndPos, pathNodeRefs, path, straightPathOptions, maxPoints);
            if (appendStatus !== AppendVertexStatus.IN_PROGRESS) {
                const maxPointsReached = (appendStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
                let flags = FindStraightPathResultFlags.SUCCESS | FindStraightPathResultFlags.PARTIAL_PATH;
                if (maxPointsReached)
                    flags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
                return makeFindStraightPathResult(flags, path);
            }
        }
    }
    // append end point
    // attach the last poly ref if available for the end point for easier identification
    const endRef = pathNodeRefs.length > 0 ? pathNodeRefs[pathNodeRefs.length - 1] : null;
    const endAppendStatus = appendVertex(closestEndPos, endRef, StraightPathPointFlags.END, path, NodeType.POLY, maxPoints);
    const maxPointsReached = (endAppendStatus & AppendVertexStatus.MAX_POINTS_REACHED) !== 0;
    let resultFlags = FindStraightPathResultFlags.SUCCESS;
    if (maxPointsReached)
        resultFlags |= FindStraightPathResultFlags.MAX_POINTS_REACHED;
    return makeFindStraightPathResult(resultFlags, path);
};

var FindPathResultFlags;
(function (FindPathResultFlags) {
    FindPathResultFlags[FindPathResultFlags["NONE"] = 0] = "NONE";
    FindPathResultFlags[FindPathResultFlags["SUCCESS"] = 1] = "SUCCESS";
    FindPathResultFlags[FindPathResultFlags["COMPLETE_PATH"] = 2] = "COMPLETE_PATH";
    FindPathResultFlags[FindPathResultFlags["PARTIAL_PATH"] = 4] = "PARTIAL_PATH";
    FindPathResultFlags[FindPathResultFlags["MAX_POINTS_REACHED"] = 8] = "MAX_POINTS_REACHED";
    FindPathResultFlags[FindPathResultFlags["INVALID_INPUT"] = 16] = "INVALID_INPUT";
    FindPathResultFlags[FindPathResultFlags["FIND_NODE_PATH_FAILED"] = 32] = "FIND_NODE_PATH_FAILED";
    FindPathResultFlags[FindPathResultFlags["FIND_STRAIGHT_PATH_FAILED"] = 64] = "FIND_STRAIGHT_PATH_FAILED";
})(FindPathResultFlags || (FindPathResultFlags = {}));
const _findPathStartNearestPolyResult = createFindNearestPolyResult();
const _findPathEndNearestPolyResult = createFindNearestPolyResult();
/**
 * Find a path between two positions on a NavMesh.
 *
 * If the end node cannot be reached through the navigation graph,
 * the last node in the path will be the nearest the end node.
 *
 * Internally:
 * - finds the closest poly for the start and end positions with @see findNearestPoly
 * - finds a nav mesh node path with @see findNodePath
 * - finds a straight path with @see findStraightPath
 *
 * If you want more fine tuned behaviour you can call these methods directly.
 * For example, for agent movement you might want to find a node path once but regularly re-call @see findStraightPath
 *
 * @param navMesh The navigation mesh.
 * @param start The starting position in world space.
 * @param end The ending position in world space.
 * @param queryFilter The query filter.
 * @returns The result of the pathfinding operation.
 */
const findPath = (navMesh, start, end, halfExtents, queryFilter) => {
    const result = {
        success: false,
        flags: FindPathResultFlags.NONE | FindPathResultFlags.INVALID_INPUT,
        nodePathFlags: FindNodePathResultFlags.NONE,
        straightPathFlags: FindStraightPathResultFlags.NONE,
        path: [],
        startNodeRef: null,
        startPosition: [0, 0, 0],
        endNodeRef: null,
        endPosition: [0, 0, 0],
        nodePath: null,
    };
    /* find start nearest poly */
    const startNearestPolyResult = findNearestPoly(_findPathStartNearestPolyResult, navMesh, start, halfExtents, queryFilter);
    if (!startNearestPolyResult.success)
        return result;
    vec3.copy(result.startPosition, startNearestPolyResult.position);
    result.startNodeRef = startNearestPolyResult.nodeRef;
    /* find end nearest poly */
    const endNearestPolyResult = findNearestPoly(_findPathEndNearestPolyResult, navMesh, end, halfExtents, queryFilter);
    if (!endNearestPolyResult.success)
        return result;
    vec3.copy(result.endPosition, endNearestPolyResult.position);
    result.endNodeRef = endNearestPolyResult.nodeRef;
    /* find node path */
    const nodePath = findNodePath(navMesh, result.startNodeRef, result.endNodeRef, result.startPosition, result.endPosition, queryFilter);
    result.nodePath = nodePath;
    result.nodePathFlags = nodePath.flags;
    if (!nodePath.success) {
        result.flags = FindPathResultFlags.FIND_NODE_PATH_FAILED;
        return result;
    }
    /* find straight path */
    const straightPath = findStraightPath(navMesh, result.startPosition, result.endPosition, nodePath.path);
    if (!straightPath.success) {
        result.flags = FindPathResultFlags.FIND_STRAIGHT_PATH_FAILED;
        return result;
    }
    result.success = true;
    result.path = straightPath.path;
    result.straightPathFlags = straightPath.flags;
    let flags = FindPathResultFlags.SUCCESS;
    if (nodePath.flags & FindNodePathResultFlags.COMPLETE_PATH &&
        (straightPath.flags & FindStraightPathResultFlags.PARTIAL_PATH) === 0) {
        flags |= FindPathResultFlags.COMPLETE_PATH;
    }
    else {
        flags |= FindPathResultFlags.PARTIAL_PATH;
    }
    if (straightPath.flags & FindStraightPathResultFlags.MAX_POINTS_REACHED) {
        flags |= FindPathResultFlags.MAX_POINTS_REACHED;
    }
    result.flags = flags;
    return result;
};

const _findSmoothPath_delta = vec3.create();
const _findSmoothPath_moveTarget = vec3.create();
const _findSmoothPath_startNearestPolyResult = createFindNearestPolyResult();
const _findSmoothPath_endNearestPolyResult = createFindNearestPolyResult();
var FindSmoothPathResultFlags;
(function (FindSmoothPathResultFlags) {
    FindSmoothPathResultFlags[FindSmoothPathResultFlags["NONE"] = 0] = "NONE";
    FindSmoothPathResultFlags[FindSmoothPathResultFlags["SUCCESS"] = 1] = "SUCCESS";
    FindSmoothPathResultFlags[FindSmoothPathResultFlags["COMPLETE_PATH"] = 2] = "COMPLETE_PATH";
    FindSmoothPathResultFlags[FindSmoothPathResultFlags["PARTIAL_PATH"] = 4] = "PARTIAL_PATH";
    FindSmoothPathResultFlags[FindSmoothPathResultFlags["INVALID_INPUT"] = 8] = "INVALID_INPUT";
    FindSmoothPathResultFlags[FindSmoothPathResultFlags["FIND_NODE_PATH_FAILED"] = 16] = "FIND_NODE_PATH_FAILED";
    FindSmoothPathResultFlags[FindSmoothPathResultFlags["FIND_STRAIGHT_PATH_FAILED"] = 32] = "FIND_STRAIGHT_PATH_FAILED";
})(FindSmoothPathResultFlags || (FindSmoothPathResultFlags = {}));
var SmoothPathPointFlags;
(function (SmoothPathPointFlags) {
    SmoothPathPointFlags[SmoothPathPointFlags["START"] = 0] = "START";
    SmoothPathPointFlags[SmoothPathPointFlags["END"] = 1] = "END";
    SmoothPathPointFlags[SmoothPathPointFlags["OFFMESH"] = 2] = "OFFMESH";
})(SmoothPathPointFlags || (SmoothPathPointFlags = {}));
/**
 * Find a smooth path between two positions on a NavMesh.
 *
 * This method computes a smooth path by iteratively moving along the navigation
 * mesh surface using the polygon path found between start and end positions.
 * The resulting path follows the surface more naturally than a straight path.
 *
 * If the end node cannot be reached through the navigation graph,
 * the path will go as far as possible toward the target.
 *
 * Internally:
 * - finds the closest poly for the start and end positions with @see findNearestPoly
 * - finds a nav mesh node path with @see findNodePath
 * - computes a smooth path by iteratively moving along the surface with @see moveAlongSurface
 *
 * @param navMesh The navigation mesh.
 * @param start The starting position in world space.
 * @param end The ending position in world space.
 * @param halfExtents The half extents for nearest polygon queries.
 * @param queryFilter The query filter.
 * @param stepSize The step size for movement along the surface
 * @param slop The distance tolerance for reaching waypoints
 * @returns The result of the smooth pathfinding operation, with path points containing position, type, and nodeRef information.
 */
const findSmoothPath = (navMesh, start, end, halfExtents, queryFilter, stepSize, slop, maxPoints) => {
    const result = {
        success: false,
        flags: FindSmoothPathResultFlags.NONE | FindSmoothPathResultFlags.INVALID_INPUT,
        path: [],
        startNodeRef: null,
        startPosition: [0, 0, 0],
        endNodeRef: null,
        endPosition: [0, 0, 0],
        nodePath: null,
    };
    /* find start nearest poly */
    const startNearestPolyResult = findNearestPoly(_findSmoothPath_startNearestPolyResult, navMesh, start, halfExtents, queryFilter);
    if (!startNearestPolyResult.success)
        return result;
    vec3.copy(result.startPosition, startNearestPolyResult.position);
    result.startNodeRef = startNearestPolyResult.nodeRef;
    /* find end nearest poly */
    const endNearestPolyResult = findNearestPoly(_findSmoothPath_endNearestPolyResult, navMesh, end, halfExtents, queryFilter);
    if (!endNearestPolyResult.success)
        return result;
    vec3.copy(result.endPosition, endNearestPolyResult.position);
    result.endNodeRef = endNearestPolyResult.nodeRef;
    /* find node path */
    const nodePath = findNodePath(navMesh, result.startNodeRef, result.endNodeRef, result.startPosition, result.endPosition, queryFilter);
    result.nodePath = nodePath;
    if (!nodePath.success || nodePath.path.length === 0) {
        result.flags = FindSmoothPathResultFlags.FIND_NODE_PATH_FAILED;
        return result;
    }
    // iterate over the path to find a smooth path
    const iterPos = vec3.clone(result.startPosition);
    const targetPos = vec3.clone(result.endPosition);
    let polys = [...nodePath.path];
    result.path.push({
        position: vec3.clone(iterPos),
        type: NodeType.POLY,
        nodeRef: result.startNodeRef,
        flags: SmoothPathPointFlags.START,
    });
    while (polys.length > 0 && result.path.length < maxPoints) {
        // find location to steer towards
        const steerTarget = getSteerTarget(navMesh, iterPos, targetPos, slop, polys);
        if (!steerTarget.success) {
            break;
        }
        const isEndOfPath = steerTarget.steerPosFlags & StraightPathPointFlags.END;
        const isOffMeshConnection = steerTarget.steerPosFlags & StraightPathPointFlags.OFFMESH;
        // find movement delta
        const steerPos = steerTarget.steerPos;
        const delta = vec3.subtract(_findSmoothPath_delta, steerPos, iterPos);
        let len = vec3.length(delta);
        // if the steer target is the end of the path or an off-mesh connection, do not move past the location
        if ((isEndOfPath || isOffMeshConnection) && len < stepSize) {
            len = 1;
        }
        else {
            len = stepSize / len;
        }
        const moveTarget = vec3.scaleAndAdd(_findSmoothPath_moveTarget, iterPos, delta, len);
        // move along surface
        const moveAlongSurfaceResult = moveAlongSurface(navMesh, polys[0], iterPos, moveTarget, queryFilter);
        if (!moveAlongSurfaceResult.success) {
            break;
        }
        const resultPosition = moveAlongSurfaceResult.position;
        polys = mergeCorridorStartMoved(polys, moveAlongSurfaceResult.visited, 256);
        fixupShortcuts(polys, navMesh);
        vec3.copy(iterPos, resultPosition);
        // handle end of path and off-mesh links when close enough
        if (isEndOfPath && inRange(iterPos, steerTarget.steerPos, slop, 1.0)) {
            // reached end of path
            vec3.copy(iterPos, targetPos);
            if (result.path.length < maxPoints) {
                result.path.push({
                    position: vec3.clone(iterPos),
                    type: NodeType.POLY,
                    nodeRef: result.endNodeRef,
                    flags: SmoothPathPointFlags.END,
                });
            }
            break;
        }
        else if (isOffMeshConnection && inRange(iterPos, steerTarget.steerPos, slop, 1.0)) {
            // reached off-mesh connection
            const offMeshConRef = steerTarget.steerPosRef;
            // advance the path up to and over the off-mesh connection
            let prevNodeRef = null;
            let nodeRef = polys[0];
            let npos = 0;
            while (npos < polys.length && nodeRef !== offMeshConRef) {
                prevNodeRef = nodeRef;
                nodeRef = polys[npos];
                npos++;
            }
            // remove processed polys
            polys.splice(0, npos);
            // handle the off-mesh connection
            const offMeshNode = getNodeByRef(navMesh, offMeshConRef);
            const offMeshConnection = navMesh.offMeshConnections[offMeshNode.offMeshConnectionId];
            if (offMeshConnection && prevNodeRef) {
                // find the link from the previous poly to the off-mesh node to determine direction
                const prevNode = getNodeByRef(navMesh, prevNodeRef);
                let linkEdge = 0; // default to START
                for (const linkIndex of prevNode.links) {
                    const link = navMesh.links[linkIndex];
                    if (link.toNodeRef === offMeshConRef) {
                        linkEdge = link.edge;
                        break;
                    }
                }
                // use the link edge to determine direction
                // edge 0 = entering from START side, edge 1 = entering from END side
                const enteringFromStart = linkEdge === 0;
                if (result.path.length < maxPoints) {
                    result.path.push({
                        position: vec3.clone(iterPos),
                        type: NodeType.OFFMESH,
                        nodeRef: offMeshConRef,
                        flags: SmoothPathPointFlags.OFFMESH,
                    });
                    const endPosition = enteringFromStart ? offMeshConnection.end : offMeshConnection.start;
                    vec3.copy(iterPos, endPosition);
                }
            }
        }
        // store results - add a point for each iteration to create smooth path
        if (result.path.length < maxPoints) {
            // determine the current ref from the current position
            const currentNodeRef = polys.length > 0 ? polys[0] : result.endNodeRef;
            result.path.push({
                position: vec3.clone(iterPos),
                type: NodeType.POLY,
                nodeRef: currentNodeRef,
                flags: 0,
            });
        }
    }
    // compose flags
    let flags = FindSmoothPathResultFlags.SUCCESS;
    if (nodePath.flags & FindNodePathResultFlags.COMPLETE_PATH) {
        flags |= FindSmoothPathResultFlags.COMPLETE_PATH;
    }
    else if (nodePath.flags & FindNodePathResultFlags.PARTIAL_PATH) {
        flags |= FindSmoothPathResultFlags.PARTIAL_PATH;
    }
    result.success = true;
    result.flags = flags;
    return result;
};
const getSteerTarget = (navMesh, start, end, minTargetDist, pathPolys) => {
    const result = {
        success: false,
        steerPos: [0, 0, 0],
        steerPosRef: 0,
        steerPosFlags: 0,
    };
    const maxStraightPathPoints = 3;
    const straightPath = findStraightPath(navMesh, start, end, pathPolys, maxStraightPathPoints, 0);
    if (!straightPath.success || straightPath.path.length === 0) {
        return result;
    }
    // find vertex far enough to steer to
    let ns = 0;
    while (ns < straightPath.path.length) {
        const point = straightPath.path[ns];
        // stop at off-mesh link
        if (point.type === NodeType.OFFMESH) {
            break;
        }
        // if this point is far enough from start, we can steer to it
        if (!inRange(point.position, start, minTargetDist, 1000.0)) {
            break;
        }
        ns++;
    }
    // failed to find good point to steer to
    if (ns >= straightPath.path.length) {
        return result;
    }
    const steerPoint = straightPath.path[ns];
    vec3.copy(result.steerPos, steerPoint.position);
    result.steerPosRef = steerPoint.nodeRef ?? 0;
    result.steerPosFlags = steerPoint.flags;
    result.success = true;
    return result;
};
const inRange = (a, b, r, h) => {
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    const dz = b[2] - a[2];
    return dx * dx + dz * dz < r * r && Math.abs(dy) < h;
};
const mergeCorridorStartMoved = (currentPath, visited, maxPath) => {
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
    let size = Math.max(0, currentPath.length - orig);
    if (req + size > maxPath) {
        size = maxPath - req;
    }
    const newPath = [];
    // store visited polygons (in reverse order)
    for (let i = 0; i < Math.min(req, maxPath); i++) {
        newPath[i] = visited[visited.length - 1 - i];
    }
    // add remaining current path
    if (size > 0) {
        for (let i = 0; i < size; i++) {
            newPath[req + i] = currentPath[orig + i];
        }
    }
    return newPath.slice(0, req + size);
};
/**
 * This function checks if the path has a small U-turn, that is,
 * a polygon further in the path is adjacent to the first polygon
 * in the path. If that happens, a shortcut is taken.
 * This can happen if the target (T) location is at tile boundary,
 * and we're approaching it parallel to the tile edge.
 * The choice at the vertex can be arbitrary,
 *  +---+---+
 *  |:::|:::|
 *  +-S-+-T-+
 *  |:::|   | <-- the step can end up in here, resulting U-turn path.
 *  +---+---+
 */
const fixupShortcuts = (pathPolys, navMesh) => {
    if (pathPolys.length < 3) {
        return;
    }
    // Get connected polygons
    const maxNeis = 16;
    let nneis = 0;
    const neis = [];
    const firstNode = getNodeByRef(navMesh, pathPolys[0]);
    for (const linkIndex of firstNode.links) {
        const link = navMesh.links[linkIndex];
        if (link?.toNodeRef && nneis < maxNeis) {
            neis.push(link.toNodeRef);
            nneis++;
        }
    }
    // If any of the neighbour polygons is within the next few polygons
    // in the path, short cut to that polygon directly.
    const maxLookAhead = 6;
    let cut = 0;
    for (let i = Math.min(maxLookAhead, pathPolys.length) - 1; i > 1 && cut === 0; i--) {
        for (let j = 0; j < nneis; j++) {
            if (pathPolys[i] === neis[j]) {
                cut = i;
                break;
            }
        }
    }
    if (cut > 1) {
        pathPolys.splice(1, cut - 1);
    }
};

// helper to insert an interval into a sorted array
const insertInterval = (intervals, tmin, tmax, nodeRef) => {
    // Find insertion point
    let idx = 0;
    while (idx < intervals.length && tmax > intervals[idx].tmin) {
        idx++;
    }
    // Insert at the found position
    intervals.splice(idx, 0, { nodeRef: nodeRef, tmin, tmax });
};
const _findLocalNeighbourhoodPolyVerticesA = [];
const _findLocalNeighbourhoodPolyVerticesB = [];
const _findLocalNeighbourhood_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
/**
 * Finds all polygons within a radius of a center position, avoiding overlapping polygons.
 *
 * This method is optimized for a small search radius and small number of result polygons.
 * Candidate polygons are found by searching the navigation graph beginning at the start polygon.
 *
 * The value of the center point is used as the start point for cost calculations.
 * It is not projected onto the surface of the mesh, so its y-value will affect the costs.
 *
 * Intersection tests occur in 2D. All polygons and the search circle are projected onto
 * the xz-plane. So the y-value of the center point does not affect intersection tests.
 *
 * @param navMesh The navigation mesh
 * @param startNodeRef The reference ID of the starting polygon
 * @param position The center position of the search circle
 * @param radius The search radius
 * @param filter The query filter to apply
 * @returns The result containing found polygons and their parents
 */
const findLocalNeighbourhood = (navMesh, startNodeRef, position, radius, filter) => {
    // search state - use SearchNodePool for this algorithm
    const nodes = {};
    const stack = [];
    const result = {
        success: false,
        nodeRefs: [],
        searchNodes: nodes,
    };
    // validate input
    if (!isValidNodeRef(navMesh, startNodeRef) || !vec3.finite(position) || radius < 0 || !Number.isFinite(radius) || !filter) {
        return result;
    }
    // initialize start node
    const startNode = {
        cost: 0,
        total: 0,
        parentNodeRef: null,
        parentState: null,
        nodeRef: startNodeRef,
        state: 0,
        flags: NODE_FLAG_CLOSED,
        position: [position[0], position[1], position[2]],
    };
    addSearchNode(nodes, startNode);
    stack.push(startNode);
    const radiusSqr = radius * radius;
    // add start polygon to results
    result.nodeRefs.push(startNodeRef);
    // temporary arrays for polygon vertices
    const polyVerticesA = _findLocalNeighbourhoodPolyVerticesA;
    const polyVerticesB = _findLocalNeighbourhoodPolyVerticesB;
    while (stack.length > 0) {
        // pop front (breadth-first search)
        const curNode = stack.shift();
        const curRef = curNode.nodeRef;
        // get current poly and tile
        const curTileAndPoly = getTileAndPolyByRef(curRef, navMesh);
        if (!curTileAndPoly.success)
            continue;
        // iterate through all links
        const node = getNodeByRef(navMesh, curRef);
        for (const linkIndex of node.links) {
            const link = navMesh.links[linkIndex];
            if (!link || !link.toNodeRef)
                continue;
            const neighbourRef = link.toNodeRef;
            // skip if already visited
            const existingNode = getSearchNode(nodes, neighbourRef, 0);
            if (existingNode && existingNode.flags & NODE_FLAG_CLOSED)
                continue;
            // get neighbour poly and tile
            const neighbourTileAndPoly = getTileAndPolyByRef(neighbourRef, navMesh);
            if (!neighbourTileAndPoly.success)
                continue;
            const { tile: neighbourTile, poly: neighbourPoly } = neighbourTileAndPoly;
            // skip off-mesh connections
            if (getNodeRefType(neighbourRef) === NodeType.OFFMESH)
                continue;
            // apply filter
            if (!filter.passFilter(neighbourRef, navMesh))
                continue;
            // find edge and calc distance to the edge
            const va = vec3.create();
            const vb = vec3.create();
            if (!getPortalPoints(navMesh, curRef, neighbourRef, va, vb))
                continue;
            // if the circle is not touching the next polygon, skip it
            distancePtSegSqr2d(_findLocalNeighbourhood_distancePtSegSqr2dResult, position, va, vb);
            if (_findLocalNeighbourhood_distancePtSegSqr2dResult.distSqr > radiusSqr)
                continue;
            // mark node visited before overlap test
            const neighbourNode = {
                cost: 0,
                total: 0,
                parentNodeRef: curRef,
                parentState: 0,
                nodeRef: neighbourRef,
                state: 0,
                flags: NODE_FLAG_CLOSED,
                position: [position[0], position[1], position[2]],
            };
            addSearchNode(nodes, neighbourNode);
            // check that the polygon does not collide with existing polygons
            // collect vertices of the neighbour poly
            const npa = neighbourPoly.vertices.length;
            for (let k = 0; k < npa; ++k) {
                const start = neighbourPoly.vertices[k] * 3;
                polyVerticesA[k * 3] = neighbourTile.vertices[start];
                polyVerticesA[k * 3 + 1] = neighbourTile.vertices[start + 1];
                polyVerticesA[k * 3 + 2] = neighbourTile.vertices[start + 2];
            }
            let overlap = false;
            for (let j = 0; j < result.nodeRefs.length; ++j) {
                const pastRef = result.nodeRefs[j];
                // connected polys do not overlap
                let connected = false;
                const curNode = getNodeByRef(navMesh, curRef);
                for (const pastLinkIndex of curNode.links) {
                    if (navMesh.links[pastLinkIndex]?.toNodeRef === pastRef) {
                        connected = true;
                        break;
                    }
                }
                if (connected)
                    continue;
                // potentially overlapping - get vertices and test overlap
                const pastTileAndPoly = getTileAndPolyByRef(pastRef, navMesh);
                if (!pastTileAndPoly.success)
                    continue;
                const { tile: pastTile, poly: pastPoly } = pastTileAndPoly;
                const npb = pastPoly.vertices.length;
                for (let k = 0; k < npb; ++k) {
                    const start = pastPoly.vertices[k] * 3;
                    polyVerticesB[k * 3] = pastTile.vertices[start];
                    polyVerticesB[k * 3 + 1] = pastTile.vertices[start + 1];
                    polyVerticesB[k * 3 + 2] = pastTile.vertices[start + 2];
                }
                if (overlapPolyPoly2D(polyVerticesA, npa, polyVerticesB, npb)) {
                    overlap = true;
                    break;
                }
            }
            if (overlap)
                continue;
            // this poly is fine, store and advance to the poly
            result.nodeRefs.push(neighbourRef);
            // add to stack for further exploration
            stack.push(neighbourNode);
        }
    }
    result.success = true;
    return result;
};
/**
 * Returns the wall segments of a polygon, optionally including portal segments.
 *
 * If segmentRefs is requested, then all polygon segments will be returned.
 * Otherwise only the wall segments are returned.
 *
 * A segment that is normally a portal will be included in the result set as a
 * wall if the filter results in the neighbor polygon becoming impassable.
 *
 * @param navMesh The navigation mesh
 * @param nodeRef The reference ID of the polygon
 * @param filter The query filter to apply
 * @param includePortals Whether to include portal segments in the result
 */
const getPolyWallSegments = (navMesh, nodeRef, filter, includePortals) => {
    const result = {
        success: false,
        segmentVerts: [],
        segmentRefs: [],
    };
    // validate input
    const tileAndPoly = getTileAndPolyByRef(nodeRef, navMesh);
    if (!tileAndPoly.success || !filter) {
        return result;
    }
    const { tile, poly } = tileAndPoly;
    const segmentVerts = result.segmentVerts;
    const segmentRefs = result.segmentRefs;
    // process each edge of the polygon
    for (let i = 0, j = poly.vertices.length - 1; i < poly.vertices.length; j = i++) {
        const intervals = [];
        // check if this edge has external links (tile boundary)
        if (poly.neis[j] & POLY_NEIS_FLAG_EXT_LINK) {
            // tile border - find all links for this edge
            const node = getNodeByRef(navMesh, nodeRef);
            for (const linkIndex of node.links) {
                const link = navMesh.links[linkIndex];
                if (!link || link.edge !== j)
                    continue;
                if (link.toNodeRef) {
                    const neighbourTileAndPoly = getTileAndPolyByRef(link.toNodeRef, navMesh);
                    if (neighbourTileAndPoly.success) {
                        if (filter.passFilter(link.toNodeRef, navMesh)) {
                            insertInterval(intervals, link.bmin, link.bmax, link.toNodeRef);
                        }
                    }
                }
            }
        }
        else {
            // internal edge
            let neiRef = null;
            if (poly.neis[j]) {
                const idx = poly.neis[j] - 1;
                neiRef = getNodeByTileAndPoly(navMesh, tile, idx).ref;
                // check if neighbor passes filter
                const neighbourTileAndPoly = getTileAndPolyByRef(neiRef, navMesh);
                if (neighbourTileAndPoly.success) {
                    if (!filter.passFilter(neiRef, navMesh)) {
                        neiRef = null;
                    }
                }
            }
            // If the edge leads to another polygon and portals are not stored, skip.
            if (neiRef !== null && !includePortals) {
                continue;
            }
            // add the full edge as a segment
            const vj = vec3.fromBuffer(vec3.create(), tile.vertices, poly.vertices[j] * 3);
            const vi = vec3.fromBuffer(vec3.create(), tile.vertices, poly.vertices[i] * 3);
            segmentVerts.push(vj[0], vj[1], vj[2], vi[0], vi[1], vi[2]);
            segmentRefs.push(neiRef);
            continue;
        }
        // add sentinels for interval processing
        insertInterval(intervals, -1, 0, null);
        insertInterval(intervals, 255, 256, null);
        // store segments based on intervals
        const vj = vec3.fromBuffer(vec3.create(), tile.vertices, poly.vertices[j] * 3);
        const vi = vec3.fromBuffer(vec3.create(), tile.vertices, poly.vertices[i] * 3);
        for (let k = 1; k < intervals.length; ++k) {
            // portal segment
            if (includePortals && intervals[k].nodeRef) {
                const tmin = intervals[k].tmin / 255.0;
                const tmax = intervals[k].tmax / 255.0;
                const segStart = vec3.create();
                const segEnd = vec3.create();
                vec3.lerp(segStart, vj, vi, tmin);
                vec3.lerp(segEnd, vj, vi, tmax);
                segmentVerts.push(segStart[0], segStart[1], segStart[2], segEnd[0], segEnd[1], segEnd[2]);
                segmentRefs.push(intervals[k].nodeRef);
            }
            // wall segment
            const imin = intervals[k - 1].tmax;
            const imax = intervals[k].tmin;
            if (imin !== imax) {
                const tmin = imin / 255.0;
                const tmax = imax / 255.0;
                const segStart = vec3.create();
                const segEnd = vec3.create();
                vec3.lerp(segStart, vj, vi, tmin);
                vec3.lerp(segEnd, vj, vi, tmax);
                segmentVerts.push(segStart[0], segStart[1], segStart[2], segEnd[0], segEnd[1], segEnd[2]);
                segmentRefs.push(null);
            }
        }
    }
    result.success = true;
    return result;
};

// debug primitive types
var DebugPrimitiveType;
(function (DebugPrimitiveType) {
    DebugPrimitiveType[DebugPrimitiveType["Triangles"] = 0] = "Triangles";
    DebugPrimitiveType[DebugPrimitiveType["Lines"] = 1] = "Lines";
    DebugPrimitiveType[DebugPrimitiveType["Points"] = 2] = "Points";
    DebugPrimitiveType[DebugPrimitiveType["Boxes"] = 3] = "Boxes";
})(DebugPrimitiveType || (DebugPrimitiveType = {}));
const hslToRgb = (out, h, s, l) => {
    h /= 360;
    const a = s * Math.min(l, 1 - l);
    const f = (n) => {
        const k = (n + h * 12) % 12;
        return l - a * Math.max(Math.min(k - 3, 9 - k, 1), -1);
    };
    out[0] = f(0);
    out[1] = f(8);
    out[2] = f(4);
    return out;
};
const regionToColor = (out, regionId, alpha = 1.0) => {
    if (regionId === 0) {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        return out;
    }
    const hash = regionId * 137.5;
    const hue = hash % 360;
    hslToRgb(out, hue, 0.7, 0.6);
    out[0] *= alpha;
    out[1] *= alpha;
    out[2] *= alpha;
    return out;
};
const areaToColor = (out, area, alpha = 1.0) => {
    if (area === WALKABLE_AREA) {
        out[0] = 0;
        out[1] = 192 / 255;
        out[2] = 1;
        return out;
    }
    if (area === NULL_AREA) {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        return out;
    }
    const hash = area * 137.5;
    const hue = hash % 360;
    hslToRgb(out, hue, 0.7, 0.6);
    out[0] *= alpha;
    out[1] *= alpha;
    out[2] *= alpha;
    return out;
};
const _color = vec3.create();
const createTriangleAreaIdsHelper = (input, triAreaIds) => {
    const areaToColorMap = {};
    const positions = [];
    const indices = [];
    const vertexColors = [];
    let positionsIndex = 0;
    let indicesIndex = 0;
    let vertexColorsIndex = 0;
    for (let triangle = 0; triangle < input.indices.length / 3; triangle++) {
        const areaId = triAreaIds[triangle];
        let color = areaToColorMap[areaId];
        if (!color) {
            if (areaId === WALKABLE_AREA) {
                color = [0, 1, 0];
            }
            else if (areaId === NULL_AREA) {
                color = [1, 0, 0];
            }
            else {
                areaToColor(_color, areaId);
                color = [_color[0], _color[1], _color[2]];
            }
            areaToColorMap[areaId] = color;
        }
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3] * 3];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3] * 3 + 1];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3] * 3 + 2];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3 + 1] * 3];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3 + 1] * 3 + 1];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3 + 1] * 3 + 2];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3 + 2] * 3];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3 + 2] * 3 + 1];
        positions[positionsIndex++] = input.positions[input.indices[triangle * 3 + 2] * 3 + 2];
        indices[indicesIndex++] = triangle * 3;
        indices[indicesIndex++] = triangle * 3 + 1;
        indices[indicesIndex++] = triangle * 3 + 2;
        for (let i = 0; i < 3; i++) {
            vertexColors[vertexColorsIndex++] = color[0];
            vertexColors[vertexColorsIndex++] = color[1];
            vertexColors[vertexColorsIndex++] = color[2];
        }
    }
    if (positions.length === 0) {
        return [];
    }
    return [
        {
            type: DebugPrimitiveType.Triangles,
            positions: positions,
            colors: vertexColors,
            indices: indices,
            transparent: true,
            opacity: 1,
        },
    ];
};
const createHeightfieldHelper = (heightfield) => {
    // Count total spans
    let totalSpans = 0;
    for (let z = 0; z < heightfield.height; z++) {
        for (let x = 0; x < heightfield.width; x++) {
            const columnIndex = x + z * heightfield.width;
            let span = heightfield.spans[columnIndex];
            while (span) {
                totalSpans++;
                span = span.next || null;
            }
        }
    }
    if (totalSpans === 0) {
        return [];
    }
    const positions = [];
    const colors = [];
    const scales = [];
    const heightfieldBoundsMin = heightfield.bounds[0];
    const cellSize = heightfield.cellSize;
    const cellHeight = heightfield.cellHeight;
    const areaToColorMap = {};
    for (let z = 0; z < heightfield.height; z++) {
        for (let x = 0; x < heightfield.width; x++) {
            const columnIndex = x + z * heightfield.width;
            let span = heightfield.spans[columnIndex];
            while (span) {
                const worldX = heightfieldBoundsMin[0] + (x + 0.5) * cellSize;
                const worldZ = heightfieldBoundsMin[2] + (z + 0.5) * cellSize;
                const spanHeight = (span.max - span.min) * cellHeight;
                const worldY = heightfieldBoundsMin[1] + (span.min + (span.max - span.min) * 0.5) * cellHeight;
                positions.push(worldX, worldY, worldZ);
                scales.push(cellSize * 0.9, spanHeight, cellSize * 0.9);
                let color = areaToColorMap[span.area];
                if (!color) {
                    if (span.area === WALKABLE_AREA) {
                        color = [0, 1, 0];
                    }
                    else if (span.area === NULL_AREA) {
                        color = [1, 0, 0];
                    }
                    else {
                        areaToColor(_color, span.area);
                        color = [_color[0], _color[1], _color[2]];
                    }
                    areaToColorMap[span.area] = color;
                }
                colors.push(color[0], color[1], color[2]);
                span = span.next || null;
            }
        }
    }
    return [
        {
            type: DebugPrimitiveType.Boxes,
            positions,
            colors,
            scales,
        },
    ];
};
const createCompactHeightfieldSolidHelper = (compactHeightfield) => {
    const chf = compactHeightfield;
    let totalQuads = 0;
    for (let y = 0; y < chf.height; y++) {
        for (let x = 0; x < chf.width; x++) {
            const cell = chf.cells[x + y * chf.width];
            totalQuads += cell.count;
        }
    }
    if (totalQuads === 0) {
        return [];
    }
    const positions = [];
    const indices = [];
    const colors = [];
    let indexOffset = 0;
    for (let y = 0; y < chf.height; y++) {
        for (let x = 0; x < chf.width; x++) {
            const fx = chf.bounds[0][0] + x * chf.cellSize;
            const fz = chf.bounds[0][2] + y * chf.cellSize;
            const cell = chf.cells[x + y * chf.width];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                const span = chf.spans[i];
                const area = chf.areas[i];
                areaToColor(_color, area);
                const fy = chf.bounds[0][1] + (span.y + 1) * chf.cellHeight;
                // Create quad vertices
                positions.push(fx, fy, fz);
                colors.push(_color[0], _color[1], _color[2]);
                positions.push(fx, fy, fz + chf.cellSize);
                colors.push(_color[0], _color[1], _color[2]);
                positions.push(fx + chf.cellSize, fy, fz + chf.cellSize);
                colors.push(_color[0], _color[1], _color[2]);
                positions.push(fx + chf.cellSize, fy, fz);
                colors.push(_color[0], _color[1], _color[2]);
                // Create triangles
                indices.push(indexOffset, indexOffset + 1, indexOffset + 2);
                indices.push(indexOffset, indexOffset + 2, indexOffset + 3);
                indexOffset += 4;
            }
        }
    }
    return [
        {
            type: DebugPrimitiveType.Triangles,
            positions: positions,
            colors: colors,
            indices: indices,
            transparent: true,
            opacity: 0.6,
            doubleSided: true,
        },
    ];
};
const createCompactHeightfieldDistancesHelper = (compactHeightfield) => {
    const chf = compactHeightfield;
    if (!chf.distances) {
        return [];
    }
    let maxd = chf.maxDistance;
    if (maxd < 1.0)
        maxd = 1;
    const dscale = 255.0 / maxd;
    let totalQuads = 0;
    for (let y = 0; y < chf.height; y++) {
        for (let x = 0; x < chf.width; x++) {
            const cell = chf.cells[x + y * chf.width];
            totalQuads += cell.count;
        }
    }
    if (totalQuads === 0) {
        return [];
    }
    const positions = [];
    const indices = [];
    const colors = [];
    let indexOffset = 0;
    for (let y = 0; y < chf.height; y++) {
        for (let x = 0; x < chf.width; x++) {
            const fx = chf.bounds[0][0] + x * chf.cellSize;
            const fz = chf.bounds[0][2] + y * chf.cellSize;
            const cell = chf.cells[x + y * chf.width];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                const span = chf.spans[i];
                const fy = chf.bounds[0][1] + (span.y + 1) * chf.cellHeight;
                const cd = Math.min(255, Math.floor(chf.distances[i] * dscale)) / 255.0;
                // Create quad vertices
                positions.push(fx, fy, fz);
                colors.push(cd, cd, cd);
                positions.push(fx, fy, fz + chf.cellSize);
                colors.push(cd, cd, cd);
                positions.push(fx + chf.cellSize, fy, fz + chf.cellSize);
                colors.push(cd, cd, cd);
                positions.push(fx + chf.cellSize, fy, fz);
                colors.push(cd, cd, cd);
                // Create triangles
                indices.push(indexOffset, indexOffset + 1, indexOffset + 2);
                indices.push(indexOffset, indexOffset + 2, indexOffset + 3);
                indexOffset += 4;
            }
        }
    }
    return [
        {
            type: DebugPrimitiveType.Triangles,
            positions: positions,
            colors: colors,
            indices: indices,
            transparent: true,
            opacity: 0.8,
            doubleSided: true,
        },
    ];
};
const createCompactHeightfieldRegionsHelper = (compactHeightfield) => {
    const chf = compactHeightfield;
    let totalQuads = 0;
    for (let y = 0; y < chf.height; y++) {
        for (let x = 0; x < chf.width; x++) {
            const cell = chf.cells[x + y * chf.width];
            totalQuads += cell.count;
        }
    }
    if (totalQuads === 0) {
        return [];
    }
    const positions = [];
    const indices = [];
    const colors = [];
    let indexOffset = 0;
    for (let y = 0; y < chf.height; y++) {
        for (let x = 0; x < chf.width; x++) {
            const fx = chf.bounds[0][0] + x * chf.cellSize;
            const fz = chf.bounds[0][2] + y * chf.cellSize;
            const cell = chf.cells[x + y * chf.width];
            for (let i = cell.index; i < cell.index + cell.count; i++) {
                const span = chf.spans[i];
                const fy = chf.bounds[0][1] + span.y * chf.cellHeight;
                regionToColor(_color, span.region);
                // Create quad vertices
                positions.push(fx, fy, fz);
                colors.push(_color[0], _color[1], _color[2]);
                positions.push(fx, fy, fz + chf.cellSize);
                colors.push(_color[0], _color[1], _color[2]);
                positions.push(fx + chf.cellSize, fy, fz + chf.cellSize);
                colors.push(_color[0], _color[1], _color[2]);
                positions.push(fx + chf.cellSize, fy, fz);
                colors.push(_color[0], _color[1], _color[2]);
                // Create triangles
                indices.push(indexOffset, indexOffset + 1, indexOffset + 2);
                indices.push(indexOffset, indexOffset + 2, indexOffset + 3);
                indexOffset += 4;
            }
        }
    }
    return [
        {
            type: DebugPrimitiveType.Triangles,
            positions: positions,
            colors: colors,
            indices: indices,
            transparent: true,
            opacity: 0.9,
            doubleSided: true,
        },
    ];
};
const createRawContoursHelper = (contourSet) => {
    if (!contourSet || contourSet.contours.length === 0) {
        return [];
    }
    const orig = contourSet.bounds[0];
    const cs = contourSet.cellSize;
    const ch = contourSet.cellHeight;
    const linePositions = [];
    const lineColors = [];
    const pointPositions = [];
    const pointColors = [];
    // Draw lines for each contour
    for (let i = 0; i < contourSet.contours.length; ++i) {
        const c = contourSet.contours[i];
        regionToColor(_color, c.reg, 0.8);
        for (let j = 0; j < c.nRawVertices; ++j) {
            const v = c.rawVertices.slice(j * 4, j * 4 + 4);
            const fx = orig[0] + v[0] * cs;
            const fy = orig[1] + (v[1] + 1 + (i & 1)) * ch;
            const fz = orig[2] + v[2] * cs;
            linePositions.push(fx, fy, fz);
            lineColors.push(_color[0], _color[1], _color[2]);
            if (j > 0) {
                linePositions.push(fx, fy, fz);
                lineColors.push(_color[0], _color[1], _color[2]);
            }
        }
        // Loop last segment
        if (c.nRawVertices > 0) {
            const v = c.rawVertices.slice(0, 4);
            const fx = orig[0] + v[0] * cs;
            const fy = orig[1] + (v[1] + 1 + (i & 1)) * ch;
            const fz = orig[2] + v[2] * cs;
            linePositions.push(fx, fy, fz);
            lineColors.push(_color[0], _color[1], _color[2]);
        }
    }
    // Draw points for each contour
    for (let i = 0; i < contourSet.contours.length; ++i) {
        const c = contourSet.contours[i];
        regionToColor(_color, c.reg, 0.8);
        const darkenedColor = [_color[0] * 0.5, _color[1] * 0.5, _color[2] * 0.5];
        for (let j = 0; j < c.nRawVertices; ++j) {
            const v = c.rawVertices.slice(j * 4, j * 4 + 4);
            let off = 0;
            let colv = darkenedColor;
            if (v[3] & 0x10000) {
                // BORDER_VERTEX
                colv = [1, 1, 1];
                off = ch * 2;
            }
            const fx = orig[0] + v[0] * cs;
            const fy = orig[1] + (v[1] + 1 + (i & 1)) * ch + off;
            const fz = orig[2] + v[2] * cs;
            pointPositions.push(fx, fy, fz);
            pointColors.push(colv[0], colv[1], colv[2]);
        }
    }
    const primitives = [];
    if (linePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: linePositions,
            colors: lineColors,
            transparent: true,
            opacity: 0.8,
            lineWidth: 2.0,
        });
    }
    if (pointPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Points,
            positions: pointPositions,
            colors: pointColors,
            size: 0.01,
            transparent: true,
        });
    }
    return primitives;
};
const createSimplifiedContoursHelper = (contourSet) => {
    if (!contourSet || contourSet.contours.length === 0) {
        return [];
    }
    const orig = contourSet.bounds[0];
    const cs = contourSet.cellSize;
    const ch = contourSet.cellHeight;
    const linePositions = [];
    const lineColors = [];
    const pointPositions = [];
    const pointColors = [];
    // Draw lines for each contour
    for (let i = 0; i < contourSet.contours.length; ++i) {
        const c = contourSet.contours[i];
        if (c.nVertices === 0)
            continue;
        regionToColor(_color, c.reg, 0.8);
        const baseColor = [_color[0], _color[1], _color[2]];
        const whiteColor = [1, 1, 1];
        // Compute border color (lerp between baseColor and white at t=128)
        const f = 128 / 255.0;
        const borderColor = [
            baseColor[0] * (1 - f) + whiteColor[0] * f,
            baseColor[1] * (1 - f) + whiteColor[1] * f,
            baseColor[2] * (1 - f) + whiteColor[2] * f,
        ];
        for (let j = 0, k = c.nVertices - 1; j < c.nVertices; k = j++) {
            const va = c.vertices.slice(k * 4, k * 4 + 4);
            const vb = c.vertices.slice(j * 4, j * 4 + 4);
            const isAreaBorder = (va[3] & 0x20000) !== 0;
            const col = isAreaBorder ? borderColor : baseColor;
            const fx1 = orig[0] + va[0] * cs;
            const fy1 = orig[1] + (va[1] + 1 + (i & 1)) * ch;
            const fz1 = orig[2] + va[2] * cs;
            const fx2 = orig[0] + vb[0] * cs;
            const fy2 = orig[1] + (vb[1] + 1 + (i & 1)) * ch;
            const fz2 = orig[2] + vb[2] * cs;
            linePositions.push(fx1, fy1, fz1);
            lineColors.push(col[0], col[1], col[2]);
            linePositions.push(fx2, fy2, fz2);
            lineColors.push(col[0], col[1], col[2]);
        }
    }
    // Draw points for each contour
    for (let i = 0; i < contourSet.contours.length; ++i) {
        const c = contourSet.contours[i];
        regionToColor(_color, c.reg, 0.8);
        const darkenedColor = [_color[0] * 0.5, _color[1] * 0.5, _color[2] * 0.5];
        for (let j = 0; j < c.nVertices; ++j) {
            const v = c.vertices.slice(j * 4, j * 4 + 4);
            let off = 0;
            let colv = darkenedColor;
            if (v[3] & 0x10000) {
                colv = [1, 1, 1];
                off = ch * 2;
            }
            const fx = orig[0] + v[0] * cs;
            const fy = orig[1] + (v[1] + 1 + (i & 1)) * ch + off;
            const fz = orig[2] + v[2] * cs;
            pointPositions.push(fx, fy, fz);
            pointColors.push(colv[0], colv[1], colv[2]);
        }
    }
    const primitives = [];
    if (linePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: linePositions,
            colors: lineColors,
            transparent: true,
            opacity: 0.9,
            lineWidth: 2.5,
        });
    }
    if (pointPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Points,
            positions: pointPositions,
            colors: pointColors,
            size: 0.01,
            transparent: true,
        });
    }
    return primitives;
};
const createPolyMeshHelper = (polyMesh) => {
    if (!polyMesh || polyMesh.nPolys === 0) {
        return [];
    }
    const nvp = polyMesh.maxVerticesPerPoly;
    const cs = polyMesh.cellSize;
    const ch = polyMesh.cellHeight;
    const orig = polyMesh.bounds[0];
    const triPositions = [];
    const triColors = [];
    const triIndices = [];
    const edgeLinePositions = [];
    const edgeLineColors = [];
    const vertexPositions = [];
    const vertexColors = [];
    let triVertexIndex = 0;
    // Draw polygon triangles
    for (let i = 0; i < polyMesh.nPolys; i++) {
        const polyBase = i * nvp;
        const area = polyMesh.areas[i];
        areaToColor(_color, area);
        // Triangulate polygon by creating a triangle fan from vertex 0
        for (let j = 2; j < nvp; j++) {
            const v0 = polyMesh.polys[polyBase + 0];
            const v1 = polyMesh.polys[polyBase + j - 1];
            const v2 = polyMesh.polys[polyBase + j];
            if (v2 === MESH_NULL_IDX)
                break;
            // Add triangle vertices
            const vertices = [v0, v1, v2];
            for (let k = 0; k < 3; k++) {
                const vertIndex = vertices[k] * 3;
                const x = orig[0] + polyMesh.vertices[vertIndex] * cs;
                const y = orig[1] + (polyMesh.vertices[vertIndex + 1] + 1) * ch;
                const z = orig[2] + polyMesh.vertices[vertIndex + 2] * cs;
                triPositions.push(x, y, z);
                triColors.push(_color[0], _color[1], _color[2]);
            }
            triIndices.push(triVertexIndex, triVertexIndex + 1, triVertexIndex + 2);
            triVertexIndex += 3;
        }
    }
    // Draw edges
    const edgeColor = [0, 48 / 255, 64 / 255];
    for (let i = 0; i < polyMesh.nPolys; i++) {
        const polyBase = i * nvp;
        for (let j = 0; j < nvp; j++) {
            const v0 = polyMesh.polys[polyBase + j];
            if (v0 === MESH_NULL_IDX)
                break;
            const nj = j + 1 >= nvp || polyMesh.polys[polyBase + j + 1] === MESH_NULL_IDX ? 0 : j + 1;
            const v1 = polyMesh.polys[polyBase + nj];
            const vertices = [v0, v1];
            for (let k = 0; k < 2; k++) {
                const vertIndex = vertices[k] * 3;
                const x = orig[0] + polyMesh.vertices[vertIndex] * cs;
                const y = orig[1] + (polyMesh.vertices[vertIndex + 1] + 1) * ch + 0.01;
                const z = orig[2] + polyMesh.vertices[vertIndex + 2] * cs;
                edgeLinePositions.push(x, y, z);
                edgeLineColors.push(edgeColor[0], edgeColor[1], edgeColor[2]);
            }
        }
    }
    // Draw vertices (points)
    const vertexColor = [1, 1, 1];
    for (let i = 0; i < polyMesh.nVertices; i++) {
        const vertIndex = i * 3;
        const x = orig[0] + polyMesh.vertices[vertIndex] * cs;
        const y = orig[1] + (polyMesh.vertices[vertIndex + 1] + 1) * ch + 0.01;
        const z = orig[2] + polyMesh.vertices[vertIndex + 2] * cs;
        vertexPositions.push(x, y, z);
        vertexColors.push(vertexColor[0], vertexColor[1], vertexColor[2]);
    }
    const primitives = [];
    if (triPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Triangles,
            positions: triPositions,
            colors: triColors,
            indices: triIndices,
            transparent: false,
            opacity: 1.0,
            doubleSided: true,
        });
    }
    if (edgeLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: edgeLinePositions,
            colors: edgeLineColors,
            transparent: true,
            opacity: 0.5,
            lineWidth: 1.5,
        });
    }
    if (vertexPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Points,
            positions: vertexPositions,
            colors: vertexColors,
            size: 0.025,
            transparent: true,
        });
    }
    return primitives;
};
const createPolyMeshDetailHelper = (polyMeshDetail) => {
    if (!polyMeshDetail || polyMeshDetail.nMeshes === 0) {
        return [];
    }
    const primitives = [];
    const edgeColor = [0, 0, 0];
    const vertexColor = [1, 1, 1];
    const submeshToColor = (out, submeshIndex) => {
        const hash = submeshIndex * 137.5;
        const hue = hash % 360;
        hslToRgb(out, hue, 0.7, 0.6);
        out[0] *= 0.3;
        out[1] *= 0.3;
        out[2] *= 0.3;
        return out;
    };
    // 1. Draw triangles
    const triPositions = [];
    const triColors = [];
    const triIndices = [];
    let triVertexIndex = 0;
    for (let i = 0; i < polyMeshDetail.nMeshes; ++i) {
        const m = i * 4;
        const bverts = polyMeshDetail.meshes[m + 0];
        const btris = polyMeshDetail.meshes[m + 2];
        const ntris = polyMeshDetail.meshes[m + 3];
        const verts = bverts * 3;
        const tris = btris * 4;
        submeshToColor(_color, i);
        for (let j = 0; j < ntris; ++j) {
            const triBase = tris + j * 4;
            const t0 = polyMeshDetail.triangles[triBase + 0];
            const t1 = polyMeshDetail.triangles[triBase + 1];
            const t2 = polyMeshDetail.triangles[triBase + 2];
            // Add triangle vertices
            const v0Base = verts + t0 * 3;
            const v1Base = verts + t1 * 3;
            const v2Base = verts + t2 * 3;
            triPositions.push(polyMeshDetail.vertices[v0Base], polyMeshDetail.vertices[v0Base + 1], polyMeshDetail.vertices[v0Base + 2], polyMeshDetail.vertices[v1Base], polyMeshDetail.vertices[v1Base + 1], polyMeshDetail.vertices[v1Base + 2], polyMeshDetail.vertices[v2Base], polyMeshDetail.vertices[v2Base + 1], polyMeshDetail.vertices[v2Base + 2]);
            // Add colors for all three vertices
            for (let k = 0; k < 3; k++) {
                triColors.push(_color[0], _color[1], _color[2]);
            }
            triIndices.push(triVertexIndex, triVertexIndex + 1, triVertexIndex + 2);
            triVertexIndex += 3;
        }
    }
    if (triPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Triangles,
            positions: triPositions,
            colors: triColors,
            indices: triIndices,
            transparent: false,
            opacity: 1.0,
        });
    }
    // 2. Draw internal edges
    const internalLinePositions = [];
    const internalLineColors = [];
    for (let i = 0; i < polyMeshDetail.nMeshes; ++i) {
        const m = i * 4;
        const bverts = polyMeshDetail.meshes[m + 0];
        const btris = polyMeshDetail.meshes[m + 2];
        const ntris = polyMeshDetail.meshes[m + 3];
        const verts = bverts * 3;
        const tris = btris * 4;
        for (let j = 0; j < ntris; ++j) {
            const t = tris + j * 4;
            const triVertices = [
                polyMeshDetail.triangles[t + 0],
                polyMeshDetail.triangles[t + 1],
                polyMeshDetail.triangles[t + 2],
            ];
            for (let k = 0, kp = 2; k < 3; kp = k++) {
                const ef = (polyMeshDetail.triangles[t + 3] >> (kp * 2)) & 0x3;
                if (ef === 0) {
                    // Internal edge
                    const tkp = triVertices[kp];
                    const tk = triVertices[k];
                    if (tkp < tk) {
                        const vkpBase = verts + tkp * 3;
                        const vkBase = verts + tk * 3;
                        internalLinePositions.push(polyMeshDetail.vertices[vkpBase], polyMeshDetail.vertices[vkpBase + 1], polyMeshDetail.vertices[vkpBase + 2], polyMeshDetail.vertices[vkBase], polyMeshDetail.vertices[vkBase + 1], polyMeshDetail.vertices[vkBase + 2]);
                        // Add colors for both endpoints
                        for (let l = 0; l < 2; l++) {
                            internalLineColors.push(edgeColor[0], edgeColor[1], edgeColor[2]);
                        }
                    }
                }
            }
        }
    }
    if (internalLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: internalLinePositions,
            colors: internalLineColors,
            transparent: true,
            opacity: 0.4,
            lineWidth: 1.0,
        });
    }
    // 3. Draw external edges
    const externalLinePositions = [];
    const externalLineColors = [];
    for (let i = 0; i < polyMeshDetail.nMeshes; ++i) {
        const m = i * 4;
        const bverts = polyMeshDetail.meshes[m + 0];
        const btris = polyMeshDetail.meshes[m + 2];
        const ntris = polyMeshDetail.meshes[m + 3];
        const verts = bverts * 3;
        const tris = btris * 4;
        for (let j = 0; j < ntris; ++j) {
            const t = tris + j * 4;
            const triVertices = [
                polyMeshDetail.triangles[t + 0],
                polyMeshDetail.triangles[t + 1],
                polyMeshDetail.triangles[t + 2],
            ];
            for (let k = 0, kp = 2; k < 3; kp = k++) {
                const ef = (polyMeshDetail.triangles[t + 3] >> (kp * 2)) & 0x3;
                if (ef !== 0) {
                    // External edge
                    const tkp = triVertices[kp];
                    const tk = triVertices[k];
                    const vkpBase = verts + tkp * 3;
                    const vkBase = verts + tk * 3;
                    externalLinePositions.push(polyMeshDetail.vertices[vkpBase], polyMeshDetail.vertices[vkpBase + 1], polyMeshDetail.vertices[vkpBase + 2], polyMeshDetail.vertices[vkBase], polyMeshDetail.vertices[vkBase + 1], polyMeshDetail.vertices[vkBase + 2]);
                    // Add colors for both endpoints
                    for (let l = 0; l < 2; l++) {
                        externalLineColors.push(edgeColor[0], edgeColor[1], edgeColor[2]);
                    }
                }
            }
        }
    }
    if (externalLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: externalLinePositions,
            colors: externalLineColors,
            transparent: true,
            opacity: 0.8,
            lineWidth: 10.0,
        });
    }
    // 4. Draw vertices as points
    const vertexPositions = [];
    const vertexColors = [];
    for (let i = 0; i < polyMeshDetail.nMeshes; ++i) {
        const m = i * 4;
        const bverts = polyMeshDetail.meshes[m];
        const nverts = polyMeshDetail.meshes[m + 1];
        const verts = bverts * 3;
        for (let j = 0; j < nverts; ++j) {
            const vBase = verts + j * 3;
            vertexPositions.push(polyMeshDetail.vertices[vBase], polyMeshDetail.vertices[vBase + 1], polyMeshDetail.vertices[vBase + 2]);
            vertexColors.push(vertexColor[0], vertexColor[1], vertexColor[2]);
        }
    }
    if (vertexPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Points,
            positions: vertexPositions,
            colors: vertexColors,
            size: 0.025,
        });
    }
    return primitives;
};
const createNavMeshHelper = (navMesh) => {
    const primitives = [];
    const triPositions = [];
    const triColors = [];
    const triIndices = [];
    let triVertexIndex = 0;
    const interPolyLinePositions = [];
    const interPolyLineColors = [];
    const outerPolyLinePositions = [];
    const outerPolyLineColors = [];
    const vertexPositions = [];
    const vertexColors = [];
    for (const tileId in navMesh.tiles) {
        const tile = navMesh.tiles[tileId];
        if (!tile)
            continue;
        // Draw detail triangles for each polygon
        for (const polyId in tile.polys) {
            const poly = tile.polys[polyId];
            const polyDetail = tile.detailMeshes[polyId];
            if (!polyDetail)
                continue;
            // Get polygon color based on area
            areaToColor(_color, poly.area, 0.4);
            const col = [_color[0], _color[1], _color[2]];
            // Draw detail triangles for this polygon
            for (let j = 0; j < polyDetail.trianglesCount; j++) {
                const triBase = (polyDetail.trianglesBase + j) * 4;
                const t0 = tile.detailTriangles[triBase];
                const t1 = tile.detailTriangles[triBase + 1];
                const t2 = tile.detailTriangles[triBase + 2];
                // Get triangle vertices
                for (let k = 0; k < 3; k++) {
                    const vertIndex = k === 0 ? t0 : k === 1 ? t1 : t2;
                    let vx;
                    let vy;
                    let vz;
                    if (vertIndex < poly.vertices.length) {
                        // Vertex from main polygon
                        const polyVertIndex = poly.vertices[vertIndex];
                        const vBase = polyVertIndex * 3;
                        vx = tile.vertices[vBase];
                        vy = tile.vertices[vBase + 1];
                        vz = tile.vertices[vBase + 2];
                    }
                    else {
                        // Vertex from detail mesh
                        const detailVertIndex = (polyDetail.verticesBase + vertIndex - poly.vertices.length) * 3;
                        vx = tile.detailVertices[detailVertIndex];
                        vy = tile.detailVertices[detailVertIndex + 1];
                        vz = tile.detailVertices[detailVertIndex + 2];
                    }
                    triPositions.push(vx, vy, vz);
                    triColors.push(...col);
                }
                // Add triangle indices
                triIndices.push(triVertexIndex, triVertexIndex + 1, triVertexIndex + 2);
                triVertexIndex += 3;
            }
        }
        // Draw polygon boundaries
        const innerColor = [0.2, 0.2, 0.2];
        const outerColor = [0.6, 0.6, 1];
        for (const polyId in tile.polys) {
            const poly = tile.polys[polyId];
            for (let j = 0; j < poly.vertices.length; j++) {
                const nj = (j + 1) % poly.vertices.length;
                const nei = poly.neis[j];
                // Check if this is a boundary edge
                const isBoundary = (nei & POLY_NEIS_FLAG_EXT_LINK) !== 0;
                // Get edge vertices
                const polyVertIndex1 = poly.vertices[j];
                const polyVertIndex2 = poly.vertices[nj];
                const v1Base = polyVertIndex1 * 3;
                const v2Base = polyVertIndex2 * 3;
                const v1x = tile.vertices[v1Base];
                const v1y = tile.vertices[v1Base + 1] + 0.01; // Slightly offset up
                const v1z = tile.vertices[v1Base + 2];
                const v2x = tile.vertices[v2Base];
                const v2y = tile.vertices[v2Base + 1] + 0.01;
                const v2z = tile.vertices[v2Base + 2];
                if (isBoundary) {
                    // Outer boundary edge
                    outerPolyLinePositions.push(v1x, v1y, v1z);
                    outerPolyLineColors.push(outerColor[0], outerColor[1], outerColor[2]);
                    outerPolyLinePositions.push(v2x, v2y, v2z);
                    outerPolyLineColors.push(outerColor[0], outerColor[1], outerColor[2]);
                }
                else {
                    // Inner polygon edge
                    interPolyLinePositions.push(v1x, v1y, v1z);
                    interPolyLineColors.push(innerColor[0], innerColor[1], innerColor[2]);
                    interPolyLinePositions.push(v2x, v2y, v2z);
                    interPolyLineColors.push(innerColor[0], innerColor[1], innerColor[2]);
                }
            }
        }
        // Draw vertices
        const vertexColor = [1, 1, 1];
        for (let i = 0; i < tile.vertices.length; i += 3) {
            const worldX = tile.vertices[i];
            const worldY = tile.vertices[i + 1];
            const worldZ = tile.vertices[i + 2];
            vertexPositions.push(worldX, worldY, worldZ);
            vertexColors.push(vertexColor[0], vertexColor[1], vertexColor[2]);
        }
    }
    // Add triangle mesh primitive
    if (triPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Triangles,
            positions: triPositions,
            colors: triColors,
            indices: triIndices,
            transparent: true,
            opacity: 0.8,
            doubleSided: true,
        });
    }
    // Add inter-poly boundary lines
    if (interPolyLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: interPolyLinePositions,
            colors: interPolyLineColors,
            transparent: true,
            opacity: 0.3,
            lineWidth: 1.5,
        });
    }
    // Add outer poly boundary lines
    if (outerPolyLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: outerPolyLinePositions,
            colors: outerPolyLineColors,
            transparent: true,
            opacity: 0.9,
            lineWidth: 2.5,
        });
    }
    // Add vertex points
    if (vertexPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Points,
            positions: vertexPositions,
            colors: vertexColors,
            size: 0.025,
        });
    }
    return primitives;
};
const createNavMeshTileHelper = (tile) => {
    const primitives = [];
    const triPositions = [];
    const triColors = [];
    const triIndices = [];
    let triVertexIndex = 0;
    const interPolyLinePositions = [];
    const interPolyLineColors = [];
    const outerPolyLinePositions = [];
    const outerPolyLineColors = [];
    const vertexPositions = [];
    const vertexColors = [];
    // Draw detail triangles for each polygon
    for (const polyId in tile.polys) {
        const poly = tile.polys[polyId];
        const polyDetail = tile.detailMeshes[polyId];
        if (!polyDetail)
            continue;
        // Get polygon color based on area
        areaToColor(_color, poly.area, 0.4);
        const col = [_color[0], _color[1], _color[2]];
        // Draw detail triangles for this polygon
        for (let j = 0; j < polyDetail.trianglesCount; j++) {
            const triBase = (polyDetail.trianglesBase + j) * 4;
            const t0 = tile.detailTriangles[triBase];
            const t1 = tile.detailTriangles[triBase + 1];
            const t2 = tile.detailTriangles[triBase + 2];
            // Get triangle vertices
            for (let k = 0; k < 3; k++) {
                const vertIndex = k === 0 ? t0 : k === 1 ? t1 : t2;
                let vx;
                let vy;
                let vz;
                if (vertIndex < poly.vertices.length) {
                    // Vertex from main polygon
                    const polyVertIndex = poly.vertices[vertIndex];
                    const vBase = polyVertIndex * 3;
                    vx = tile.vertices[vBase];
                    vy = tile.vertices[vBase + 1];
                    vz = tile.vertices[vBase + 2];
                }
                else {
                    // Vertex from detail mesh
                    const detailVertIndex = (polyDetail.verticesBase + vertIndex - poly.vertices.length) * 3;
                    vx = tile.detailVertices[detailVertIndex];
                    vy = tile.detailVertices[detailVertIndex + 1];
                    vz = tile.detailVertices[detailVertIndex + 2];
                }
                triPositions.push(vx, vy, vz);
                triColors.push(...col);
            }
            // Add triangle indices
            triIndices.push(triVertexIndex, triVertexIndex + 1, triVertexIndex + 2);
            triVertexIndex += 3;
        }
    }
    // Draw polygon boundaries
    const innerColor = [0.2, 0.2, 0.2];
    const outerColor = [0.6, 0.6, 1];
    for (const polyId in tile.polys) {
        const poly = tile.polys[polyId];
        for (let j = 0; j < poly.vertices.length; j++) {
            const nj = (j + 1) % poly.vertices.length;
            const nei = poly.neis[j];
            // Check if this is a boundary edge
            const isBoundary = (nei & POLY_NEIS_FLAG_EXT_LINK) !== 0;
            // Get edge vertices
            const polyVertIndex1 = poly.vertices[j];
            const polyVertIndex2 = poly.vertices[nj];
            const v1Base = polyVertIndex1 * 3;
            const v2Base = polyVertIndex2 * 3;
            const v1x = tile.vertices[v1Base];
            const v1y = tile.vertices[v1Base + 1] + 0.01; // Slightly offset up
            const v1z = tile.vertices[v1Base + 2];
            const v2x = tile.vertices[v2Base];
            const v2y = tile.vertices[v2Base + 1] + 0.01;
            const v2z = tile.vertices[v2Base + 2];
            if (isBoundary) {
                // Outer boundary edge
                outerPolyLinePositions.push(v1x, v1y, v1z);
                outerPolyLineColors.push(outerColor[0], outerColor[1], outerColor[2]);
                outerPolyLinePositions.push(v2x, v2y, v2z);
                outerPolyLineColors.push(outerColor[0], outerColor[1], outerColor[2]);
            }
            else {
                // Inner polygon edge
                interPolyLinePositions.push(v1x, v1y, v1z);
                interPolyLineColors.push(innerColor[0], innerColor[1], innerColor[2]);
                interPolyLinePositions.push(v2x, v2y, v2z);
                interPolyLineColors.push(innerColor[0], innerColor[1], innerColor[2]);
            }
        }
    }
    // Draw vertices
    const vertexColor = [1, 1, 1];
    for (let i = 0; i < tile.vertices.length; i += 3) {
        const worldX = tile.vertices[i];
        const worldY = tile.vertices[i + 1];
        const worldZ = tile.vertices[i + 2];
        vertexPositions.push(worldX, worldY, worldZ);
        vertexColors.push(vertexColor[0], vertexColor[1], vertexColor[2]);
    }
    // Add triangle mesh primitive
    if (triPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Triangles,
            positions: triPositions,
            colors: triColors,
            indices: triIndices,
            transparent: true,
            opacity: 0.8,
            doubleSided: true,
        });
    }
    // Add inter-poly boundary lines
    if (interPolyLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: interPolyLinePositions,
            colors: interPolyLineColors,
            transparent: true,
            opacity: 0.3,
            lineWidth: 1.5,
        });
    }
    // Add outer poly boundary lines
    if (outerPolyLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: outerPolyLinePositions,
            colors: outerPolyLineColors,
            transparent: true,
            opacity: 0.9,
            lineWidth: 2.5,
        });
    }
    // Add vertex points
    if (vertexPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Points,
            positions: vertexPositions,
            colors: vertexColors,
            size: 0.025,
        });
    }
    return primitives;
};
const createNavMeshPolyHelper = (navMesh, nodeRef, color = [0, 0.75, 1]) => {
    const primitives = [];
    // Get tile and polygon from reference
    const { tileId, polyIndex } = getNodeByRef(navMesh, nodeRef);
    const tile = navMesh.tiles[tileId];
    if (!tile || !tile.polys[polyIndex]) {
        // Return empty array if polygon not found
        return primitives;
    }
    const poly = tile.polys[polyIndex];
    // Get the detail mesh for this polygon
    const detailMesh = tile.detailMeshes?.[polyIndex];
    if (!detailMesh) {
        // Fallback: draw basic polygon without detail mesh
        const triPositions = [];
        const triColors = [];
        const triIndices = [];
        // Create a simple triangle fan from the polygon vertices
        if (poly.vertices.length >= 3) {
            const baseColor = [color[0] * 0.25, color[1] * 0.25, color[2] * 0.25]; // Transparent
            for (let i = 2; i < poly.vertices.length; i++) {
                const v0Index = poly.vertices[0] * 3;
                const v1Index = poly.vertices[i - 1] * 3;
                const v2Index = poly.vertices[i] * 3;
                // Add triangle vertices
                triPositions.push(tile.vertices[v0Index], tile.vertices[v0Index + 1], tile.vertices[v0Index + 2]);
                triPositions.push(tile.vertices[v1Index], tile.vertices[v1Index + 1], tile.vertices[v1Index + 2]);
                triPositions.push(tile.vertices[v2Index], tile.vertices[v2Index + 1], tile.vertices[v2Index + 2]);
                // Add colors
                for (let j = 0; j < 3; j++) {
                    triColors.push(baseColor[0], baseColor[1], baseColor[2]);
                }
                // Add indices
                const baseIndex = (i - 2) * 3;
                triIndices.push(baseIndex, baseIndex + 1, baseIndex + 2);
            }
        }
        if (triPositions.length > 0) {
            primitives.push({
                type: DebugPrimitiveType.Triangles,
                positions: triPositions,
                colors: triColors,
                indices: triIndices,
                transparent: true,
                opacity: 0.6,
                doubleSided: true,
            });
        }
        return primitives;
    }
    // Draw detail triangles for this polygon
    const triPositions = [];
    const triColors = [];
    const triIndices = [];
    const baseColor = [color[0] * 0.25, color[1] * 0.25, color[2] * 0.25]; // Make color transparent
    for (let i = 0; i < detailMesh.trianglesCount; ++i) {
        const t = (detailMesh.trianglesBase + i) * 4;
        const detailTriangles = tile.detailTriangles;
        for (let j = 0; j < 3; ++j) {
            const vertIndex = detailTriangles[t + j];
            if (vertIndex < poly.vertices.length) {
                const polyVertIndex = poly.vertices[vertIndex] * 3;
                triPositions.push(tile.vertices[polyVertIndex], tile.vertices[polyVertIndex + 1], tile.vertices[polyVertIndex + 2]);
            }
            else {
                const detailVertIndex = (detailMesh.verticesBase + vertIndex - poly.vertices.length) * 3;
                triPositions.push(tile.detailVertices[detailVertIndex], tile.detailVertices[detailVertIndex + 1], tile.detailVertices[detailVertIndex + 2]);
            }
            triColors.push(baseColor[0], baseColor[1], baseColor[2]);
        }
        const baseIndex = i * 3;
        triIndices.push(baseIndex, baseIndex + 1, baseIndex + 2);
    }
    if (triPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Triangles,
            positions: triPositions,
            colors: triColors,
            indices: triIndices,
            transparent: true,
            opacity: 0.6,
            doubleSided: true,
        });
    }
    return primitives;
};
const createNavMeshTileBvTreeHelper = (navMeshTile) => {
    const primitives = [];
    if (navMeshTile.bvTree.nodes.length === 0) {
        return primitives;
    }
    // Arrays for wireframe box edges
    const linePositions = [];
    const lineColors = [];
    // Color for BV tree nodes (white with transparency)
    const nodeColor = [1, 1, 1];
    // Calculate inverse quantization factor (cs = 1.0f / tile->header->bvQuantFactor)
    const cs = 1.0 / navMeshTile.bvTree.quantFactor;
    // Draw BV nodes - only internal nodes (leaf indices are positive, internal are negative)
    for (let i = 0; i < navMeshTile.bvTree.nodes.length; i++) {
        const node = navMeshTile.bvTree.nodes[i];
        // Leaf indices are positive.
        if (node.i < 0)
            continue;
        // Calculate world coordinates from quantized bounds
        const minX = navMeshTile.bounds[0][0] + node.bounds[0][0] * cs;
        const minY = navMeshTile.bounds[0][1] + node.bounds[0][1] * cs;
        const minZ = navMeshTile.bounds[0][2] + node.bounds[0][2] * cs;
        const maxX = navMeshTile.bounds[0][0] + node.bounds[1][0] * cs;
        const maxY = navMeshTile.bounds[0][1] + node.bounds[1][1] * cs;
        const maxZ = navMeshTile.bounds[0][2] + node.bounds[1][2] * cs;
        // Create wireframe box edges
        // Bottom face
        linePositions.push(minX, minY, minZ, maxX, minY, minZ);
        linePositions.push(maxX, minY, minZ, maxX, minY, maxZ);
        linePositions.push(maxX, minY, maxZ, minX, minY, maxZ);
        linePositions.push(minX, minY, maxZ, minX, minY, minZ);
        // Top face
        linePositions.push(minX, maxY, minZ, maxX, maxY, minZ);
        linePositions.push(maxX, maxY, minZ, maxX, maxY, maxZ);
        linePositions.push(maxX, maxY, maxZ, minX, maxY, maxZ);
        linePositions.push(minX, maxY, maxZ, minX, maxY, minZ);
        // Vertical edges
        linePositions.push(minX, minY, minZ, minX, maxY, minZ);
        linePositions.push(maxX, minY, minZ, maxX, maxY, minZ);
        linePositions.push(maxX, minY, maxZ, maxX, maxY, maxZ);
        linePositions.push(minX, minY, maxZ, minX, maxY, maxZ);
        // Add colors for all line segments (24 vertices = 12 line segments)
        for (let j = 0; j < 24; j++) {
            lineColors.push(nodeColor[0], nodeColor[1], nodeColor[2]);
        }
    }
    // Create line segments primitive
    if (linePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: linePositions,
            colors: lineColors,
            transparent: true,
            opacity: 0.5,
            lineWidth: 1.0,
        });
    }
    return primitives;
};
const createNavMeshBvTreeHelper = (navMesh) => {
    const primitives = [];
    // Draw BV tree for all tiles in the nav mesh
    for (const tileId in navMesh.tiles) {
        const tile = navMesh.tiles[tileId];
        if (!tile)
            continue;
        const tilePrimitives = createNavMeshTileBvTreeHelper(tile);
        primitives.push(...tilePrimitives);
    }
    return primitives;
};
const _createNavMeshLinksHelper_sourceCenter = vec3.create();
const _createNavMeshLinksHelper_targetCenter = vec3.create();
const _createNavMeshLinksHelper_edgeStart = vec3.create();
const _createNavMeshLinksHelper_edgeEnd = vec3.create();
const _createNavMeshLinksHelper_edgeMidpoint = vec3.create();
const _createNavMeshLinksHelper_sourcePoint = vec3.create();
const _createNavMeshLinksHelper_targetPoint = vec3.create();
const createNavMeshLinksHelper = (navMesh) => {
    const primitives = [];
    // Arrays for line data
    const linePositions = [];
    const lineColors = [];
    // Color for navmesh links
    const linkColor = [1, 1, 0]; // Bright yellow
    // Helper function to lerp between two values
    const lerp = (start, end, t) => start + (end - start) * t;
    // Helper function to calculate polygon center
    const getPolyCenter = (out, tile, poly) => {
        let centerX = 0;
        let centerY = 0;
        let centerZ = 0;
        const nv = poly.vertices.length;
        for (let i = 0; i < nv; i++) {
            const vertIndex = poly.vertices[i] * 3;
            centerX += tile.vertices[vertIndex];
            centerY += tile.vertices[vertIndex + 1];
            centerZ += tile.vertices[vertIndex + 2];
        }
        out[0] = centerX / nv;
        out[1] = centerY / nv;
        out[2] = centerZ / nv;
        return out;
    };
    // Process each link
    for (const link of navMesh.links) {
        if (!link.allocated)
            continue;
        // Get source polygon info
        const { tileId: sourceTileId, polyIndex: sourcePolyId } = navMesh.nodes[link.fromNodeIndex];
        const sourceTile = navMesh.tiles[sourceTileId];
        const sourcePoly = sourceTile?.polys[sourcePolyId];
        // Get target polygon info
        const { tileId: targetTileId, polyIndex: targetPolyId } = navMesh.nodes[link.toNodeIndex];
        const targetTile = navMesh.tiles[targetTileId];
        const targetPoly = targetTile?.polys[targetPolyId];
        if (!sourceTile || !sourcePoly || !targetTile || !targetPoly) {
            continue;
        }
        // Calculate polygon centers
        const sourceCenter = getPolyCenter(_createNavMeshLinksHelper_sourceCenter, sourceTile, sourcePoly);
        const targetCenter = getPolyCenter(_createNavMeshLinksHelper_targetCenter, targetTile, targetPoly);
        // Get the edge vertices for this link
        const edgeIndex = link.edge;
        const nextEdgeIndex = (edgeIndex + 1) % sourcePoly.vertices.length;
        const v0Index = sourcePoly.vertices[edgeIndex] * 3;
        const v1Index = sourcePoly.vertices[nextEdgeIndex] * 3;
        const edgeStart = vec3.fromBuffer(_createNavMeshLinksHelper_edgeStart, sourceTile.vertices, v0Index);
        const edgeEnd = vec3.fromBuffer(_createNavMeshLinksHelper_edgeEnd, sourceTile.vertices, v1Index);
        // Calculate edge midpoint
        const edgeMidpoint = vec3.add(_createNavMeshLinksHelper_edgeMidpoint, edgeStart, edgeEnd);
        vec3.scale(edgeMidpoint, edgeMidpoint, 0.5);
        // Move the edge midpoint slightly towards the polygon center (10% of the way)
        const inwardFactor = 0.1;
        const sourcePoint = vec3.lerp(_createNavMeshLinksHelper_sourcePoint, edgeMidpoint, sourceCenter, inwardFactor);
        sourcePoint[1] += 0.05; // slight y offset
        // For the target, use the target polygon center
        const targetPoint = vec3.copy(_createNavMeshLinksHelper_targetPoint, targetCenter);
        targetPoint[1] += 0.05; // slight y offset
        // Create arced line with multiple segments
        const numSegments = 12;
        const arcHeight = 0.3;
        for (let i = 0; i < numSegments; i++) {
            const t0 = i / numSegments;
            const t1 = (i + 1) / numSegments;
            // Calculate positions along arc with sinusoidal height
            const x0 = lerp(sourcePoint[0], targetPoint[0], t0);
            const y0 = lerp(sourcePoint[1], targetPoint[1], t0) + Math.sin(t0 * Math.PI) * arcHeight;
            const z0 = lerp(sourcePoint[2], targetPoint[2], t0);
            const x1 = lerp(sourcePoint[0], targetPoint[0], t1);
            const y1 = lerp(sourcePoint[1], targetPoint[1], t1) + Math.sin(t1 * Math.PI) * arcHeight;
            const z1 = lerp(sourcePoint[2], targetPoint[2], t1);
            // Add line segment
            linePositions.push(x0, y0, z0, x1, y1, z1);
            // Add colors for both endpoints
            lineColors.push(linkColor[0], linkColor[1], linkColor[2]);
            lineColors.push(linkColor[0], linkColor[1], linkColor[2]);
        }
        // Add a simple arrow at the end to show direction
        const arrowT = 0.85; // Position arrow near the end
        const arrowX = lerp(sourcePoint[0], targetPoint[0], arrowT);
        const arrowY = lerp(sourcePoint[1], targetPoint[1], arrowT) + Math.sin(arrowT * Math.PI) * arcHeight;
        const arrowZ = lerp(sourcePoint[2], targetPoint[2], arrowT);
        // Calculate direction for arrow
        const nextT = 0.95;
        const nextX = lerp(sourcePoint[0], targetPoint[0], nextT);
        const nextY = lerp(sourcePoint[1], targetPoint[1], nextT) + Math.sin(nextT * Math.PI) * arcHeight;
        const nextZ = lerp(sourcePoint[2], targetPoint[2], nextT);
        const dirX = nextX - arrowX;
        const dirY = nextY - arrowY;
        const dirZ = nextZ - arrowZ;
        const len = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ) || 1;
        const nx = dirX / len;
        const ny = dirY / len;
        const nz = dirZ / len;
        // Create arrow head with two lines
        const arrowLength = 0.15;
        const arrowWidth = 0.08;
        // Calculate perpendicular vectors for arrow wings
        const perpX = -nz;
        const perpZ = nx;
        // Left wing
        linePositions.push(arrowX, arrowY, arrowZ, arrowX - nx * arrowLength + perpX * arrowWidth, arrowY - ny * arrowLength, arrowZ - nz * arrowLength + perpZ * arrowWidth);
        // Right wing
        linePositions.push(arrowX, arrowY, arrowZ, arrowX - nx * arrowLength - perpX * arrowWidth, arrowY - ny * arrowLength, arrowZ - nz * arrowLength - perpZ * arrowWidth);
        // Add colors for arrow (4 vertices = 2 line segments)
        for (let k = 0; k < 4; k++) {
            lineColors.push(linkColor[0], linkColor[1], linkColor[2]);
        }
    }
    if (linePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: linePositions,
            colors: lineColors,
            transparent: true,
            opacity: 0.9,
            lineWidth: 3.0,
        });
    }
    return primitives;
};
const createNavMeshTilePortalsHelper = (navMeshTile) => {
    const primitives = [];
    const padx = 0.04; // (purely visual)
    const pady = navMeshTile.walkableClimb; // vertical extent
    const sideColors = {
        0: [128 / 255, 0, 0], // red
        2: [0, 128 / 255, 0], // green
        4: [128 / 255, 0, 128 / 255], // magenta
        6: [0, 128 / 255, 128 / 255], // cyan
    };
    const positions = [];
    const colors = [];
    const drawSides = [0, 2, 4, 6];
    for (const side of drawSides) {
        const matchMask = POLY_NEIS_FLAG_EXT_LINK | side;
        const color = sideColors[side];
        if (!color)
            continue;
        for (const polyId in navMeshTile.polys) {
            const poly = navMeshTile.polys[polyId];
            const nv = poly.vertices.length;
            for (let j = 0; j < nv; j++) {
                if (poly.neis[j] !== matchMask)
                    continue; // must be exact match
                const v0Index = poly.vertices[j];
                const v1Index = poly.vertices[(j + 1) % nv];
                const aBase = v0Index * 3;
                const bBase = v1Index * 3;
                const ax = navMeshTile.vertices[aBase];
                const ay = navMeshTile.vertices[aBase + 1];
                const az = navMeshTile.vertices[aBase + 2];
                const bx = navMeshTile.vertices[bBase];
                const by = navMeshTile.vertices[bBase + 1];
                const bz = navMeshTile.vertices[bBase + 2];
                if (side === 0 || side === 4) {
                    const x = ax + (side === 0 ? -padx : padx);
                    // Four edges of rectangle (8 vertices for 4 line segments)
                    positions.push(x, ay - pady, az, x, ay + pady, az);
                    positions.push(x, ay + pady, az, x, by + pady, bz);
                    positions.push(x, by + pady, bz, x, by - pady, bz);
                    positions.push(x, by - pady, bz, x, ay - pady, az);
                }
                else if (side === 2 || side === 6) {
                    const z = az + (side === 2 ? -padx : padx);
                    positions.push(ax, ay - pady, z, ax, ay + pady, z);
                    positions.push(ax, ay + pady, z, bx, by + pady, z);
                    positions.push(bx, by + pady, z, bx, by - pady, z);
                    positions.push(bx, by - pady, z, ax, ay - pady, z);
                }
                // Add color entries (8 vertices per rectangle)
                for (let k = 0; k < 8; k++) {
                    colors.push(color[0], color[1], color[2]);
                }
            }
        }
    }
    if (positions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions,
            colors,
            transparent: true,
            opacity: 0.5,
            lineWidth: 2.0,
        });
    }
    return primitives;
};
const createNavMeshPortalsHelper = (navMesh) => {
    const primitives = [];
    for (const tileId in navMesh.tiles) {
        const tile = navMesh.tiles[tileId];
        if (!tile)
            continue;
        const tilePrimitives = createNavMeshTilePortalsHelper(tile);
        primitives.push(...tilePrimitives);
    }
    return primitives;
};
const createSearchNodesHelper = (nodePool) => {
    const primitives = [];
    if (!nodePool || Object.keys(nodePool).length === 0) {
        return primitives;
    }
    const yOffset = 0.5;
    const pointPositions = [];
    const pointColors = [];
    const linePositions = [];
    const lineColors = [];
    // Color (255,192,0) -> (1, 0.7529, 0)
    const pointColor = [1.0, 192 / 255, 0.0];
    const lineColor = [1.0, 192 / 255, 0.0];
    // Collect all nodes from the pool (each nodeRef can have multiple states)
    for (const nodeRef in nodePool) {
        const nodes = nodePool[nodeRef];
        for (let i = 0; i < nodes.length; i++) {
            const node = nodes[i];
            const [x, y, z] = node.position;
            pointPositions.push(x, y + yOffset, z);
            pointColors.push(pointColor[0], pointColor[1], pointColor[2]);
        }
    }
    // Lines to parents
    for (const nodeRef in nodePool) {
        const nodes = nodePool[nodeRef];
        for (let i = 0; i < nodes.length; i++) {
            const node = nodes[i];
            if (node.parentNodeRef === null || node.parentState === null)
                continue;
            // Find parent node
            const parentNodes = nodePool[node.parentNodeRef];
            if (!parentNodes)
                continue;
            let parent;
            for (let j = 0; j < parentNodes.length; j++) {
                if (parentNodes[j].state === node.parentState) {
                    parent = parentNodes[j];
                    break;
                }
            }
            if (!parent)
                continue;
            const [cx, cy, cz] = node.position;
            const [px, py, pz] = parent.position;
            linePositions.push(cx, cy + yOffset, cz, px, py + yOffset, pz);
            lineColors.push(lineColor[0], lineColor[1], lineColor[2], lineColor[0], lineColor[1], lineColor[2]);
        }
    }
    if (pointPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Points,
            positions: pointPositions,
            colors: pointColors,
            size: 0.01,
            transparent: true,
            opacity: 1.0,
        });
    }
    if (linePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: linePositions,
            colors: lineColors,
            transparent: true,
            opacity: 0.5,
            lineWidth: 2.0,
        });
    }
    return primitives;
};
const createNavMeshOffMeshConnectionsHelper = (navMesh) => {
    const primitives = [];
    const arcSegments = 16;
    const circleSegments = 20;
    // Aggregated arrays for arcs & circles
    const arcPositions = [];
    const arcColors = [];
    const circlePositions = [];
    const circleColors = [];
    const arcColor = [255 / 255, 196 / 255, 0 / 255]; // main arc color
    const pillarColor = [0 / 255, 48 / 255, 64 / 255]; // pillar color
    const oneWayEndColor = [220 / 255, 32 / 255, 16 / 255]; // end marker if one-way
    // Helper function to lerp between two values
    const lerp = (start, end, t) => start + (end - start) * t;
    for (const id in navMesh.offMeshConnections) {
        const con = navMesh.offMeshConnections[id];
        if (!con)
            continue;
        const start = con.start;
        const end = con.end;
        const radius = con.radius;
        // Arc polyline (adds a vertical sinusoidal lift up to 0.25 at mid)
        for (let i = 0; i < arcSegments; i++) {
            const t0 = i / arcSegments;
            const t1 = (i + 1) / arcSegments;
            const x0 = lerp(start[0], end[0], t0);
            const y0 = lerp(start[1], end[1], t0) + Math.sin(t0 * Math.PI) * 0.25;
            const z0 = lerp(start[2], end[2], t0);
            const x1 = lerp(start[0], end[0], t1);
            const y1 = lerp(start[1], end[1], t1) + Math.sin(t1 * Math.PI) * 0.25;
            const z1 = lerp(start[2], end[2], t1);
            arcPositions.push(x0, y0, z0, x1, y1, z1);
            for (let k = 0; k < 2; k++)
                arcColors.push(arcColor[0], arcColor[1], arcColor[2]);
        }
        // One-way direction arrow in the middle of arc
        if (con.direction === OffMeshConnectionDirection.START_TO_END) {
            const tMid = 0.5;
            const xMid = lerp(start[0], end[0], tMid);
            const yMid = lerp(start[1], end[1], tMid) + 0.25;
            const zMid = lerp(start[2], end[2], tMid);
            const dirX = end[0] - start[0];
            const dirZ = end[2] - start[2];
            const len = Math.hypot(dirX, dirZ) || 1;
            const nx = dirX / len;
            const nz = dirZ / len;
            const back = 0.3;
            arcPositions.push(xMid, yMid, zMid, xMid - nx * back + nz * back * 0.5, yMid - 0.05, zMid - nz * back - nx * back * 0.5, xMid, yMid, zMid, xMid - nx * back - nz * back * 0.5, yMid - 0.05, zMid - nz * back + nx * back * 0.5);
            for (let k = 0; k < 4; k++)
                arcColors.push(arcColor[0], arcColor[1], arcColor[2]);
        }
        const addCircle = (center, color) => {
            for (let i = 0; i < circleSegments; i++) {
                const a0 = (i / circleSegments) * Math.PI * 2;
                const a1 = ((i + 1) / circleSegments) * Math.PI * 2;
                const x0 = center[0] + Math.cos(a0) * radius;
                const z0 = center[2] + Math.sin(a0) * radius;
                const x1 = center[0] + Math.cos(a1) * radius;
                const z1 = center[2] + Math.sin(a1) * radius;
                circlePositions.push(x0, center[1] + 0.1, z0, x1, center[1] + 0.1, z1);
                for (let k = 0; k < 2; k++)
                    circleColors.push(color[0], color[1], color[2]);
            }
            // Pillar
            circlePositions.push(center[0], center[1], center[2], center[0], center[1] + 0.2, center[2]);
            for (let k = 0; k < 2; k++)
                circleColors.push(pillarColor[0], pillarColor[1], pillarColor[2]);
        };
        addCircle(start, arcColor);
        addCircle(end, con.direction === OffMeshConnectionDirection.BIDIRECTIONAL ? arcColor : oneWayEndColor);
    }
    if (arcPositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: arcPositions,
            colors: arcColors,
            transparent: true,
            opacity: 0.9,
            lineWidth: 2.0,
        });
    }
    if (circlePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: circlePositions,
            colors: circleColors,
            transparent: true,
            opacity: 0.8,
            lineWidth: 1.5,
        });
    }
    return primitives;
};
// All debug helper functions are now implemented
// The pattern is: return DebugPrimitive[] where each primitive has a 'type' field

export { ANY_QUERY_FILTER, AREA_BORDER, AXIS_X, AXIS_Y, AXIS_Z, BORDER_REG, BORDER_VERTEX, BuildContext, BuildContextLogType, CONTOUR_REG_MASK, ContourBuildFlags, DEFAULT_QUERY_FILTER, DETAIL_EDGE_BOUNDARY, DIR_OFFSETS, DebugPrimitiveType, FindNodePathResultFlags, FindPathResultFlags, FindSmoothPathResultFlags, FindStraightPathOptions, FindStraightPathResultFlags, INVALID_NODE_REF, MAX_HEIGHT, MAX_LAYERS, MAX_NODE_INDEX, MAX_SEQUENCE, MESH_NULL_IDX, MULTIPLE_REGS, NODE_FLAG_CLOSED, NODE_FLAG_OPEN, NODE_FLAG_PARENT_DETACHED, NOT_CONNECTED, NULL_AREA, NodeType, OffMeshConnectionDirection, POLY_NEIS_FLAG_EXT_LINK, POLY_NEIS_FLAG_EXT_LINK_DIR_MASK, SlicedFindNodePathInitFlags, SlicedFindNodePathStatusFlags, SmoothPathPointFlags, StraightPathPointFlags, WALKABLE_AREA, addHeightfieldSpan, addOffMeshConnection, addSearchNode, addTile, bubbleUpQueue, buildCompactHeightfield, buildContours, buildDistanceField, buildLayerRegions, buildPolyMesh, buildPolyMeshDetail, buildRegions, buildRegionsMonotone, buildTile, calculateGridSize, calculateMeshBounds, clearUnwalkableTriangles, createCompactHeightfieldDistancesHelper, createCompactHeightfieldRegionsHelper, createCompactHeightfieldSolidHelper, createDefaultQueryFilter, createFindNearestPolyResult, createGetClosestPointOnPolyResult, createGetPolyHeightResult, createHeightfield, createHeightfieldHelper, createNavMesh, createNavMeshBvTreeHelper, createNavMeshHelper, createNavMeshLinksHelper, createNavMeshOffMeshConnectionsHelper, createNavMeshPolyHelper, createNavMeshPortalsHelper, createNavMeshTileBvTreeHelper, createNavMeshTileHelper, createNavMeshTilePortalsHelper, createPolyMeshDetailHelper, createPolyMeshHelper, createRawContoursHelper, createSearchNodesHelper, createSimplifiedContoursHelper, createSlicedNodePathQuery, createTriangleAreaIdsHelper, erodeAndMarkWalkableAreas, erodeWalkableArea, filterLedgeSpans, filterLowHangingWalkableObstacles, filterWalkableLowHeightSpans, finalizeSlicedFindNodePath, finalizeSlicedFindNodePathPartial, findLocalNeighbourhood, findNearestPoly, findNodePath, findPath, findRandomPoint, findRandomPointAroundCircle, findSmoothPath, findStraightPath, geometry, getClosestPointOnDetailEdges, getClosestPointOnPoly, getClosestPointOnPolyBoundary, getCon, getDirForOffset, getDirOffsetX, getDirOffsetY, getEdgeMidPoint, getNodeByRef, getNodeByTileAndPoly, getNodeRefIndex, getNodeRefSequence, getNodeRefType, getPolyHeight, getPolyWallSegments, getPortalPoints, getSearchNode, getTileAndPolyByRef, getTileAt, getTilesAt, initSlicedFindNodePath, isOffMeshConnectionConnected, isValidNodeRef, markBoxArea, markConvexPolyArea, markCylinderArea, markRotatedBoxArea, markWalkableTriangles, medianFilterWalkableArea, moveAlongSurface, polyMeshDetailToTileDetailMesh, polyMeshToTilePolys, polygonsToNavMeshTilePolys, polysToTileDetailMesh, popNodeFromQueue, pushNodeToQueue, queryPolygons, queryPolygonsInTile, rasterizeTriangles, raycast, raycastWithCosts, reconnectOffMeshConnection, reindexNodeInQueue, removeOffMeshConnection, removeTile, serNodeRef, setCon, trickleDownQueue, updateSlicedFindNodePath, worldToTilePosition };
//# sourceMappingURL=index.js.map

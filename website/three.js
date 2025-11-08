import * as NavCat from 'navcat';
import { DebugPrimitiveType, createFindNearestPolyResult } from 'navcat';
import * as THREE from 'three';
import { Vector3 } from 'three';
import { vec3, vec2 } from 'mathcat';

const primitiveToThreeJS = (primitive) => {
    const disposables = [];
    switch (primitive.type) {
        case DebugPrimitiveType.Triangles: {
            const triPrimitive = primitive;
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(triPrimitive.positions), 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(triPrimitive.colors), 3));
            if (triPrimitive.indices && triPrimitive.indices.length > 0) {
                geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(triPrimitive.indices), 1));
            }
            const material = new THREE.MeshBasicMaterial({
                vertexColors: true,
                transparent: triPrimitive.transparent || false,
                opacity: triPrimitive.opacity || 1.0,
                side: triPrimitive.doubleSided ? THREE.DoubleSide : THREE.FrontSide,
            });
            const mesh = new THREE.Mesh(geometry, material);
            disposables.push(() => {
                geometry.dispose();
                material.dispose();
            });
            return {
                object: mesh,
                dispose: () => {
                    for (const dispose of disposables) {
                        dispose();
                    }
                },
            };
        }
        case DebugPrimitiveType.Lines: {
            const linePrimitive = primitive;
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(linePrimitive.positions), 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(linePrimitive.colors), 3));
            const material = new THREE.LineBasicMaterial({
                vertexColors: true,
                transparent: linePrimitive.transparent || false,
                opacity: linePrimitive.opacity || 1.0,
                linewidth: linePrimitive.lineWidth || 1.0,
            });
            const lines = new THREE.LineSegments(geometry, material);
            disposables.push(() => {
                geometry.dispose();
                material.dispose();
            });
            return {
                object: lines,
                dispose: () => {
                    for (const dispose of disposables) {
                        dispose();
                    }
                },
            };
        }
        case DebugPrimitiveType.Points: {
            const pointPrimitive = primitive;
            const group = new THREE.Group();
            const numPoints = pointPrimitive.positions.length / 3;
            if (numPoints > 0) {
                // Create sphere geometry for instancing
                const sphereGeometry = new THREE.SphereGeometry(1, 8, 6); // Low-poly sphere for performance
                const material = new THREE.MeshBasicMaterial({
                    vertexColors: true,
                    transparent: pointPrimitive.transparent || false,
                    opacity: pointPrimitive.opacity || 1.0,
                });
                const instancedMesh = new THREE.InstancedMesh(sphereGeometry, material, numPoints);
                const matrix = new THREE.Matrix4();
                const baseSize = pointPrimitive.size || 1.0;
                for (let i = 0; i < numPoints; i++) {
                    const x = pointPrimitive.positions[i * 3];
                    const y = pointPrimitive.positions[i * 3 + 1];
                    const z = pointPrimitive.positions[i * 3 + 2];
                    const sphereSize = baseSize;
                    matrix.makeScale(sphereSize, sphereSize, sphereSize);
                    matrix.setPosition(x, y, z);
                    instancedMesh.setMatrixAt(i, matrix);
                    const color = new THREE.Color(pointPrimitive.colors[i * 3], pointPrimitive.colors[i * 3 + 1], pointPrimitive.colors[i * 3 + 2]);
                    instancedMesh.setColorAt(i, color);
                }
                instancedMesh.instanceMatrix.needsUpdate = true;
                if (instancedMesh.instanceColor) {
                    instancedMesh.instanceColor.needsUpdate = true;
                }
                group.add(instancedMesh);
                disposables.push(() => {
                    sphereGeometry.dispose();
                    material.dispose();
                    instancedMesh.dispose();
                });
            }
            return {
                object: group,
                dispose: () => {
                    for (const dispose of disposables) {
                        dispose();
                    }
                },
            };
        }
        case DebugPrimitiveType.Boxes: {
            const boxPrimitive = primitive;
            const group = new THREE.Group();
            // Create instanced mesh for all boxes
            const boxGeometry = new THREE.BoxGeometry(1, 1, 1);
            const numBoxes = boxPrimitive.positions.length / 3;
            if (numBoxes > 0) {
                const material = new THREE.MeshBasicMaterial({
                    vertexColors: true,
                    transparent: boxPrimitive.transparent || false,
                    opacity: boxPrimitive.opacity || 1.0,
                });
                const instancedMesh = new THREE.InstancedMesh(boxGeometry, material, numBoxes);
                const matrix = new THREE.Matrix4();
                for (let i = 0; i < numBoxes; i++) {
                    const x = boxPrimitive.positions[i * 3];
                    const y = boxPrimitive.positions[i * 3 + 1];
                    const z = boxPrimitive.positions[i * 3 + 2];
                    const scaleX = boxPrimitive.scales ? boxPrimitive.scales[i * 3] : 1;
                    const scaleY = boxPrimitive.scales ? boxPrimitive.scales[i * 3 + 1] : 1;
                    const scaleZ = boxPrimitive.scales ? boxPrimitive.scales[i * 3 + 2] : 1;
                    matrix.makeScale(scaleX, scaleY, scaleZ);
                    matrix.setPosition(x, y, z);
                    instancedMesh.setMatrixAt(i, matrix);
                    const color = new THREE.Color(boxPrimitive.colors[i * 3], boxPrimitive.colors[i * 3 + 1], boxPrimitive.colors[i * 3 + 2]);
                    instancedMesh.setColorAt(i, color);
                }
                instancedMesh.instanceMatrix.needsUpdate = true;
                if (instancedMesh.instanceColor) {
                    instancedMesh.instanceColor.needsUpdate = true;
                }
                group.add(instancedMesh);
                disposables.push(() => {
                    boxGeometry.dispose();
                    material.dispose();
                    instancedMesh.dispose();
                });
            }
            return {
                object: group,
                dispose: () => {
                    for (const dispose of disposables) {
                        dispose();
                    }
                },
            };
        }
        default: {
            const exhaustiveCheck = primitive;
            console.warn('Unknown debug primitive type:', exhaustiveCheck.type);
            return { object: new THREE.Group(), dispose: () => { } };
        }
    }
};
/**
 * Converts an array of debug primitives to a Three.js group
 */
function primitivesToThreeJS(primitives) {
    const group = new THREE.Group();
    const disposables = [];
    for (const primitive of primitives) {
        const { object, dispose } = primitiveToThreeJS(primitive);
        group.add(object);
        disposables.push(dispose);
    }
    return {
        object: group,
        dispose: () => {
            for (const dispose of disposables) {
                dispose();
            }
        },
    };
}
const createTriangleAreaIdsHelper = (input, triAreaIds) => {
    const primitives = NavCat.createTriangleAreaIdsHelper(input, triAreaIds);
    return primitivesToThreeJS(primitives);
};
const createHeightfieldHelper = (heightfield) => {
    const primitives = NavCat.createHeightfieldHelper(heightfield);
    return primitivesToThreeJS(primitives);
};
const createCompactHeightfieldSolidHelper = (compactHeightfield) => {
    const primitives = NavCat.createCompactHeightfieldSolidHelper(compactHeightfield);
    return primitivesToThreeJS(primitives);
};
const createCompactHeightfieldDistancesHelper = (compactHeightfield) => {
    const primitives = NavCat.createCompactHeightfieldDistancesHelper(compactHeightfield);
    return primitivesToThreeJS(primitives);
};
const createCompactHeightfieldRegionsHelper = (compactHeightfield) => {
    const primitives = NavCat.createCompactHeightfieldRegionsHelper(compactHeightfield);
    return primitivesToThreeJS(primitives);
};
const createRawContoursHelper = (contourSet) => {
    const primitives = NavCat.createRawContoursHelper(contourSet);
    return primitivesToThreeJS(primitives);
};
const createSimplifiedContoursHelper = (contourSet) => {
    const primitives = NavCat.createSimplifiedContoursHelper(contourSet);
    return primitivesToThreeJS(primitives);
};
const createPolyMeshHelper = (polyMesh) => {
    const primitives = NavCat.createPolyMeshHelper(polyMesh);
    return primitivesToThreeJS(primitives);
};
const createPolyMeshDetailHelper = (polyMeshDetail) => {
    const primitives = NavCat.createPolyMeshDetailHelper(polyMeshDetail);
    return primitivesToThreeJS(primitives);
};
const createNavMeshHelper = (navMesh) => {
    const primitives = NavCat.createNavMeshHelper(navMesh);
    return primitivesToThreeJS(primitives);
};
const createNavMeshTileHelper = (tile) => {
    const primitives = NavCat.createNavMeshTileHelper(tile);
    return primitivesToThreeJS(primitives);
};
const createNavMeshPolyHelper = (navMesh, nodeRef, color = [0, 0.75, 1]) => {
    const primitives = NavCat.createNavMeshPolyHelper(navMesh, nodeRef, color);
    return primitivesToThreeJS(primitives);
};
const createNavMeshTileBvTreeHelper = (navMeshTile) => {
    const primitives = NavCat.createNavMeshTileBvTreeHelper(navMeshTile);
    return primitivesToThreeJS(primitives);
};
const createNavMeshLinksHelper = (navMesh) => {
    const primitives = NavCat.createNavMeshLinksHelper(navMesh);
    return primitivesToThreeJS(primitives);
};
const createNavMeshBvTreeHelper = (navMesh) => {
    const primitives = NavCat.createNavMeshBvTreeHelper(navMesh);
    return primitivesToThreeJS(primitives);
};
const createNavMeshTilePortalsHelper = (navMeshTile) => {
    const primitives = NavCat.createNavMeshTilePortalsHelper(navMeshTile);
    return primitivesToThreeJS(primitives);
};
const createNavMeshPortalsHelper = (navMesh) => {
    const primitives = NavCat.createNavMeshPortalsHelper(navMesh);
    return primitivesToThreeJS(primitives);
};
const createSearchNodesHelper = (nodePool) => {
    const primitives = NavCat.createSearchNodesHelper(nodePool);
    return primitivesToThreeJS(primitives);
};
const createNavMeshOffMeshConnectionsHelper = (navMesh) => {
    const primitives = NavCat.createNavMeshOffMeshConnectionsHelper(navMesh);
    return primitivesToThreeJS(primitives);
};

vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();

vec3.create();
vec3.create();

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
createFindNearestPolyResult();
vec3.create();
vec3.create();
vec3.create();
vec3.create();
vec2.create();
vec2.create();
vec3.create();
vec3.create();
vec3.create();
vec3.create();

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

const _position = new Vector3();
const getPositionsAndIndices = (meshes) => {
    const toMerge = [];
    for (const mesh of meshes) {
        const positionAttribute = mesh.geometry.attributes.position;
        if (!positionAttribute || positionAttribute.itemSize !== 3) {
            continue;
        }
        mesh.updateMatrixWorld();
        const positions = new Float32Array(positionAttribute.array);
        for (let i = 0; i < positions.length; i += 3) {
            const pos = _position.set(positions[i], positions[i + 1], positions[i + 2]);
            mesh.localToWorld(pos);
            positions[i] = pos.x;
            positions[i + 1] = pos.y;
            positions[i + 2] = pos.z;
        }
        let indices = mesh.geometry.getIndex()?.array;
        if (indices === undefined) {
            // this will become indexed when merging with other meshes
            const ascendingIndex = [];
            for (let i = 0; i < positionAttribute.count; i++) {
                ascendingIndex.push(i);
            }
            indices = ascendingIndex;
        }
        toMerge.push({
            positions,
            indices,
        });
    }
    return mergePositionsAndIndices(toMerge);
};

export { createCompactHeightfieldDistancesHelper, createCompactHeightfieldRegionsHelper, createCompactHeightfieldSolidHelper, createHeightfieldHelper, createNavMeshBvTreeHelper, createNavMeshHelper, createNavMeshLinksHelper, createNavMeshOffMeshConnectionsHelper, createNavMeshPolyHelper, createNavMeshPortalsHelper, createNavMeshTileBvTreeHelper, createNavMeshTileHelper, createNavMeshTilePortalsHelper, createPolyMeshDetailHelper, createPolyMeshHelper, createRawContoursHelper, createSearchNodesHelper, createSimplifiedContoursHelper, createTriangleAreaIdsHelper, getPositionsAndIndices };
//# sourceMappingURL=three.js.map

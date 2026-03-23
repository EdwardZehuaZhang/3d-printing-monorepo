/*
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License
https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

Copyright (c) 2023, Takanori Fujiwara and S. Sandra Bae
All rights reserved.
*/


const socket = io();

let mode = null; // collection or demo

// DOM elements for visual state
const liveDot = document.getElementById('liveDot');
const collectionStartButton = document.getElementById('collectionStartButton');
const demoStartButton = document.getElementById('demoStartButton');
const progressBarFill = document.getElementById('progressBarFill');
const selectedNodeEl = document.getElementById('selectedNode');
let liveDotActivated = false;

// for data collection and preprocessing
const collectingData = [];
let currentCollectingNode = -1;
let startTime = null;
let intervalId = null;

// for selection demo
let config = null;
const bufferSizeForCapValues = 10;
const bufferSizeForSelectedNodes = 10;
let prevSelectedNode = -1;
const bufferedCapValues = [];
const bufferedSelectedNodes = [];

const processData = (data) => {
  const statusEl = document.querySelector("#currentCollectingNode");
  statusEl.innerHTML = "Processing collected data";
  statusEl.classList.add('active');

  // Enable demo start button once step 1 is complete
  demoStartButton.classList.remove('is-disabled');
  demoStartButton.disabled = false;

  // Activate the Node Detection panel
  document.querySelector('.card--step2').classList.add('is-active');

  const vegaLiteSpec = {
    $schema: 'https://vega.github.io/schema/vega-lite/v5.json',
    description: 'Values from Arduino.',
    width: 800,
    data: {
      values: data
    },
    mark: 'line',
    encoding: {
      x: { field: "time", type: "temporal" },
      y: { field: "value", type: "quantitative" },
      color: { field: "node", type: "nominal" }
    }
  };
  vegaEmbed("#vis", vegaLiteSpec);

  // cut first cutThres
  const cutThres = 1500 // in ms
  config = {};

  const dataByNode = {
  }
  for (const d of data) {
    const node = parseInt(d.node);
    if (!(node in dataByNode)) {
      dataByNode[node] = { 'times': [], 'values': [] };
    } else {
      dataByNode[node].times.push(parseFloat(d.time));
      dataByNode[node].values.push(parseFloat(d.value));
    }
  }
  for (const node in dataByNode) {
    let lastTime = 0;
    if (node != -1) {
      lastTime = dataByNode[node - 1].times.slice(-1)[0];
    }

    let nVals = 0;
    config[node] = 0.0;
    for (let i = 0; i < dataByNode[node].times.length; ++i) {
      const time = dataByNode[node].times[i];
      if (time >= lastTime + cutThres) {
        config[node] += dataByNode[node].values[i];
        nVals++;
      }
    }
    config[node] /= nVals;
  }

  socket.emit("endProcessing", JSON.stringify(config));
  const finishedEl = document.querySelector("#currentCollectingNode");
  finishedEl.innerHTML = "Finished processing data (config data is in data directory)";
  finishedEl.classList.add('active');
};

const selectCloseCapValNode = (aveCapValue, config) => {
  const nodes = Object.keys(config);
  const repCapValues = Object.values(config);

  let selectedNode = -1;
  let minDiff = 1e100;
  for (let i = 0; i < nodes.length; ++i) {
    const diff = Math.abs(repCapValues[i] - aveCapValue);
    if (diff < minDiff) {
      selectedNode = nodes[i]
      minDiff = diff;
    }
  }

  return parseInt(selectedNode);
}

collectionStartButton.addEventListener("click", () => {
  mode = "collection";

  socket.emit("io", {
    "statuses": "1"
  });
  startTime = performance.now();

  currentCollectingNode = -1;
  collectingData.length = 0;

  const nTotal = parseInt(document.querySelector("#nNodesInput").value);

  // Create individual progress bar segments
  const container = document.querySelector("#progressBarsContainer");
  container.innerHTML = '';
  for (let i = 0; i < nTotal; i++) {
    const segment = document.createElement('div');
    segment.className = 'progress-bar-segment';
    segment.id = `progress-segment-${i}`;
    container.appendChild(segment);
  }

  const statusEl = document.querySelector("#currentCollectingNode");
  statusEl.innerHTML = 'Initializing...';
  statusEl.classList.remove('active');
  collectionStartButton.classList.add('is-active');

  intervalId = setInterval(() => {
    currentCollectingNode++;
    if (currentCollectingNode < nTotal) {
      statusEl.innerHTML = `Touch Node ${currentCollectingNode} (${nTotal} total)`;
      statusEl.classList.add('active');
      // Fill the segment for the completed node
      if (currentCollectingNode > 0) {
        document.querySelector(`#progress-segment-${currentCollectingNode - 1}`).classList.add('filled');
      }
    } else {
      statusEl.innerHTML = "Collection complete. Processing...";
      statusEl.classList.add('active');
      // Fill the last segment
      document.querySelector(`#progress-segment-${nTotal - 1}`).classList.add('filled');
      socket.emit("endCollection", JSON.stringify(collectingData));
      clearInterval(intervalId);
      collectionStartButton.classList.remove('is-active');
      processData(collectingData);
    }
  }, 5000);
});

demoStartButton.addEventListener("click", () => {
  mode = "demo";
  demoStartButton.classList.add('is-active');

  if (config) {
    socket.emit("io", {
      "statuses": "1"
    });
  } else {
    fetch("./data/config.json")
      .then(response => response.json())
      .then(json => {
        config = json;
        socket.emit("io", {
          "statuses": "1"
        });
      });
  }
});

socket.on("data", data => {
  const capValue = data.capValue;
  const connectType = data.connectType;
  // use smaller buffer sizes when not using a serial port
  const bufferSizeDiv = connectType == 'serial' ? 1 : 2;

  // Activate live dot on first data event with refined animation
  if (!liveDotActivated) {
    liveDot.classList.add('active');
    liveDotActivated = true;
  }

  if (mode == "collection") {
    const time = performance.now() - startTime; // in millisec
    collectingData.push({
      "node": currentCollectingNode,
      "time": time,
      "value": capValue
    });
  } else {
    bufferedCapValues.push(parseFloat(capValue));
    if (bufferedCapValues.length > bufferSizeForCapValues / bufferSizeDiv) {
      bufferedCapValues.shift()
      const aveCapValue = bufferedCapValues.reduce((cum, val) => cum + val) / (bufferSizeForCapValues / bufferSizeDiv);
      const tmpSelectedNode = selectCloseCapValNode(aveCapValue, config);
      bufferedSelectedNodes.push(tmpSelectedNode);

      if (bufferedSelectedNodes.length > bufferSizeForSelectedNodes) {
        bufferedSelectedNodes.shift();
        const counts = bufferedSelectedNodes.reduce((acc, node) => {
          if (node in acc) {
            acc[node]++;
          } else {
            acc[node] = 1;
          }
          return acc;
        }, {});

        const sortedNodesByCounts = Object.keys(counts).sort((a, b) => - counts[a] + counts[b]);
        let mostFreqNode = sortedNodesByCounts[0];
        let selectedNode = -1;

        if (counts[mostFreqNode] > bufferSizeForSelectedNodes * 0.8) {
          selectedNode = mostFreqNode;
          prevSelectedNode = selectedNode;
          bufferedCapValues.length = 0;
          selectedNodeEl.classList.add('is-active');
        } else {
          selectedNode = prevSelectedNode;
        }

        document.querySelector("#selectedNode").innerHTML = `${selectedNode}`;
      }
    }
  }
});
# Autonomous Path Planning Visualizer

A web-based, interactive **Pathfinding Visualizer** built with **React** and **Tailwind CSS**. This project allows users to explore and visualize popular pathfinding algorithms in real-time on a customizable grid.

---

## 🚀 Live Demo

Explore the live version of the Pathfinding Visualizer here:

[autonomous-path-planning.vercel.app](https://autonomous-path-planning.vercel.app/)

---

## 📌 Table of Contents

* [Project Overview](#-project-overview)
* [Features](#-features)
* [Algorithms](#-algorithms)
* [Installation](#-installation)
* [Usage](#-usage)
* [Components](#-components)
* [Customization](#-customization)
* [Deployment](#-deployment)
* [License](#-license)
* [Acknowledgements](#-acknowledgements)

---

## 🧭 Project Overview

The Pathfinding Visualizer allows users to:

* Create walls and weighted cells to modify the grid.
* Set **start** and **goal** nodes for pathfinding.
* Visualize algorithms like **A\***, **Dijkstra’s**, and **RRT**\*.
* Save and load grid configurations via **JSON import/export**.
* Interact with a fully responsive grid with smooth animations.

This tool is ideal for learning and understanding pathfinding algorithms interactively.

---

## ✨ Features

* **Interactive Grid**: Click or drag to add walls and weights.
* **Start & Goal Nodes**: Easily set start and end points.
* **Real-time Visualization**: Watch algorithms traverse the grid step by step.
* **Weighted Cells**: Add custom weights to see shortest paths considering cost.
* **Import/Export Grids**: Save grids for later use or share with others.
* **Responsive Design**: Works across different screen sizes.
* **Legend & UI Controls**: Clear UI to understand the grid state and actions.

---

## 🧠 Algorithms

The visualizer includes:

1. **A\*** – Uses heuristics to efficiently find the shortest path.
2. **Dijkstra’s Algorithm** – Finds the shortest path considering weights and guarantees optimal paths.
3. **RRT\*** – Rapidly-exploring Random Tree\* algorithm for path planning in complex spaces.

---

## 🛠 Installation

1. Clone the repository:

```bash
git clone https://github.com/yourusername/pathfinding-visualizer.git
cd pathfinding-visualizer
```

2. Install dependencies:

```bash
npm install
```

3. Start the development server:

```bash
npm run dev
```

4. Open your browser at `http://localhost:5173`.

---

## 🖱 Usage

1. Click or drag on the grid to **create walls**.
2. Select **weight mode** and click cells to assign custom weights.
3. Click to **set the start and goal nodes**.
4. Choose an algorithm to **visualize pathfinding**.
5. Export the grid using **Export JSON**.
6. Load a grid using **Import JSON**.

**Tip:** Weighted paths are visible only when weights are applied on the path.

---

## 🧩 Components

* **Grid** – The main interactive grid.
* **Cell** – Individual grid cell supporting walls, weights, start, goal, and path.
* **Legend** – Shows meaning of colors (Wall, Weighted, Visited, Path).
* **Controls** – Buttons for algorithm selection, grid reset, and import/export.
* **Icons** – Lucide Icons for Start (Flag) and Goal (MapPinned).

---

## 🎨 Customization

* **Grid Size**: Adjust number of rows and columns via `rows` and `cols` state.
* **Colors**: Change Tailwind classes for walls, weights, paths, and visited cells.
* **Algorithms**: Add or modify algorithms in `src/algorithms/`.
* **Animation Speed**: Control speed of pathfinding animations by adjusting timeout/delay.

---

## 🚀 Deployment

1. Build the project:

```bash
npm run build
```

2. Deploy the `dist/` folder to any static hosting service such as Vercel, Netlify, or GitHub Pages.

**Example (GitHub Pages):**

```bash
npm run build
npx gh-pages -d dist
```

---

## 📝 License

This project is licensed under the [MIT License](LICENSE).

---

## 🙏 Acknowledgements

* Inspired by popular pathfinding visualizer tutorials and educational projects.
* Icons by [Lucide](https://lucide.dev/).
* Tailwind CSS for clean and responsive design.
* Special thanks to the open-source community for algorithms and visualization techniques.

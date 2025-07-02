import React from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import { ThemeProvider } from "styled-components";
import Home from "./pages/Home";
import Scan from "./pages/Scan";
import Demo from "./pages/Demo";
import Scans from "./pages/Scans";
import Settings from "./pages/Settings";
import Game from "./pages/Game";
import theme from "./styles/theme";
import { ScanProvider } from "./context/ScanContext";
import Viewer from "./pages/Viewer";
import EditScanModal from "./components/EditScanModal";
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';



function App() {
  return (
    <ThemeProvider theme={theme}>
      <ScanProvider>
      <Router>
        <Routes>
          <Route path="/viewer" element={<Viewer />} />
          <Route path="/" element={<Home />} />
          <Route path="/scan" element={<Scan />} />
          <Route path="/demo" element={<Demo />} />
          <Route path="/scans" element={<Scans />} />
          <Route path="/settings" element={<Settings />} />
          <Route path="/game" element={<Game />} />
          <Route path="/edit-scan" element={<EditScanModal />} />
        </Routes>
      </Router>
      </ScanProvider>
    </ThemeProvider>
  );
}

export default App;


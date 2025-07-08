import React, { createContext, useContext, useState } from "react";

const ScanContext = createContext();

export function useScans() {
  return useContext(ScanContext);
}

export function ScanProvider({ children }) {
  const [scans, setScans] = useState([]);

  const addScan = (scan) => setScans((prev) => [...prev, scan]);

  return (
    <ScanContext.Provider value={{ scans, addScan }}>
      {children}
    </ScanContext.Provider>
  );
}

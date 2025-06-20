import React from "react";
import styled from "styled-components";

const UploadBox = styled.div`
  background-color: ${(p) => p.theme.colors.panel};
  padding: 30px;
  border-radius: 12px;
  color: ${(p) => p.theme.colors.text};
  text-align: center;
`;

function ScanUpload({ onUpload }) {
  const handleChange = (e) => {
    const file = e.target.files[0];
    if (file) {
      onUpload(file);
    }
  };

  return (
    <UploadBox>
      <h3>🛰️ Simulate Drone Scan Upload</h3>
      <input type="file" accept=".stl,.obj,.glb" onChange={handleChange} />
    </UploadBox>
  );
}

export default ScanUpload;

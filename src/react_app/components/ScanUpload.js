// components/ScanUpload.js
import React, { useRef, useState } from "react";
import styled from "styled-components";

const DropArea = styled.div`
  border: 2px dashed #00ffc6;
  border-radius: 12px;
  padding: 40px;
  text-align: center;
  transition: background-color 0.3s ease;
  cursor: pointer;
  color: white;

  &:hover {
    background-color: rgba(0, 255, 198, 0.1);
  }
`;

function ScanUpload({ onUpload }) {
  const fileInputRef = useRef();
  const [dragging, setDragging] = useState(false);

  const handleDrop = (e) => {
    e.preventDefault();
    setDragging(false);
    const file = e.dataTransfer.files[0];
    if (file) onUpload(file);
  };

  const handleFileSelect = (e) => {
    const file = e.target.files[0];
    if (file) onUpload(file);
  };

  return (
    <>
      <DropArea
        onClick={() => fileInputRef.current.click()}
        onDragOver={(e) => {
          e.preventDefault();
          setDragging(true);
        }}
        onDragLeave={() => setDragging(false)}
        onDrop={handleDrop}
        style={{
          backgroundColor: dragging ? "rgba(0,255,198,0.2)" : "transparent",
        }}
      >
        <p>{dragging ? "📡 Drop your scan here" : "Drag & drop or click to upload"}</p>
      </DropArea>
      <input
        type="file"
        ref={fileInputRef}
        style={{ display: "none" }}
        onChange={handleFileSelect}
      />
    </>
  );
}

export default ScanUpload;

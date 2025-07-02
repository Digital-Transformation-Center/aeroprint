import React, { useState } from "react";
import styled from "styled-components";

const Overlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: rgba(0, 0, 0, 0.6);
  display: ${({ show }) => (show ? "flex" : "none")};
  align-items: center;
  justify-content: center;
  z-index: 999;
`;

const ModalContainer = styled.div`
  background-color: #1e1e1e;
  padding: 30px;
  border-radius: 12px;
  width: 90%;
  max-width: 500px;
  color: white;
`;

const Input = styled.input`
  width: 100%;
  margin-bottom: 20px;
  padding: 10px;
  font-size: 16px;
  border-radius: 6px;
  border: 1px solid #555;
  background-color: #2c2f36;
  color: white;
`;

const ButtonGroup = styled.div`
  display: flex;
  justify-content: flex-end;
  gap: 10px;
`;

const Button = styled.button`
  padding: 10px 18px;
  font-size: 14px;
  border-radius: 6px;
  border: none;
  cursor: pointer;
  background-color: ${({ variant }) =>
    variant === "save" ? "#00ffc6" : "#555"};
  color: ${({ variant }) => (variant === "cancel" ? "#eee" : "#000")};
`;

function EditScanModal({ show, onClose, onSave, scan }) {
  const [editedName, setEditedName] = useState(scan?.name || "");

  const handleSave = () => {
    if (!editedName.trim()) return;
    onSave({ ...scan, name: editedName });
    onClose();
  };

  if (!scan) return null;

  return (
    <Overlay show={show}>
      <ModalContainer>
        <h2>Edit Scan</h2>
        <label>Scan Name</label>
        <Input
          value={editedName}
          onChange={(e) => setEditedName(e.target.value)}
          placeholder="Scan name"
        />
        <ButtonGroup>
          <Button variant="cancel" onClick={onClose}>
            Cancel
          </Button>
          <Button variant="save" onClick={handleSave}>
            Save
          </Button>
        </ButtonGroup>
      </ModalContainer>
    </Overlay>
  );
}

export default EditScanModal;

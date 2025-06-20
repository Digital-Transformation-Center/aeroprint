import React from "react";
import styled from "styled-components";

const Card = styled.div`
  background-color: ${(p) => p.theme.colors.panel};
  border-radius: 16px;
  padding: 20px;
  text-align: center;
  cursor: pointer;
  color: ${(p) => p.theme.colors.text};
  transition: transform 0.3s ease, box-shadow 0.3s ease;

  &:hover {
    transform: scale(1.03);
    box-shadow: 0 0 20px rgba(0, 255, 198, 0.4);
  }
`;

function FeatureCard({ title, onClick, children }) {
  return (
    <Card onClick={onClick}>
      <h3>{title}</h3>
      {children}
    </Card>
  );
}

export default FeatureCard;

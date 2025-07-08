import React from "react";
import styled from "styled-components";
import { useNavigate } from "react-router-dom";
import FeatureCard from "../components/FeatureCard";
import StartButton from "../components/StartButton";

const Wrapper = styled.div`
  background-color: ${(p) => p.theme.colors.background};
  min-height: 100vh;
  padding: 40px;
  color: ${(p) => p.theme.colors.text};
`;

const Grid = styled.div`
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 24px;
`;

const Center = styled.div`
  display: flex;
  justify-content: center;
  margin-top: 40px;
`;

function Home() {
  const navigate = useNavigate();

  return (
    <Wrapper>
      <h1>🛸 AeroPrint Command Center</h1>
      <Grid>
        <FeatureCard title="Recent Scans" onClick={() => navigate("/scans")}>
          <p>View previously captured models</p>
        </FeatureCard>
        <FeatureCard title="DEMO" onClick={() => navigate("/demo")}>
          <p>Preview simulation mode</p>
        </FeatureCard>
        <FeatureCard title="Scan Quest" onClick={() => navigate("/game")}>
          <p>Gamified scan challenges</p>
        </FeatureCard>
        <FeatureCard title="Settings" onClick={() => navigate("/settings")}>
          <p>Adjust config and system paths</p>
        </FeatureCard>
      </Grid>
      <Center>
        <StartButton onClick={() => navigate("/scan")}>
          🚀 Start Scan
        </StartButton>
      </Center>
    </Wrapper>
  );
}

export default Home;

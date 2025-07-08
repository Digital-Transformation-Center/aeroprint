import styled from "styled-components";
 
const StartButton = styled.button`
  background: linear-gradient(145deg, #00ffc6, #4b8aff);
  border: none;
  padding: 20px 60px;
  font-size: 1.2rem;
  border-radius: 50px;
  color: white;
  font-family: ${(p) => p.theme.fonts.main};
  box-shadow: 0 0 25px rgba(0, 255, 198, 0.6);
  animation: pulse 2s infinite;
  cursor: pointer;
  margin-top: 40px;

  @keyframes pulse {
    0% { box-shadow: 0 0 10px rgba(0,255,198,0.4); }
    50% { box-shadow: 0 0 25px rgba(0,255,198,0.9); }
    100% { box-shadow: 0 0 10px rgba(0,255,198,0.4); }
  }
`;

export default StartButton;

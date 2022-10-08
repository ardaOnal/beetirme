import React from "react";

import {
  AppBar,
  Avatar,
  Box,
  Checkbox,
  Container,
  createTheme,
  CssBaseline,
  List,
  ListItem,
  ListItemAvatar,
  ListItemIcon,
  ListItemText,
  Paper,
  ThemeProvider,
  Toolbar,
  Typography,
} from "@mui/material";
import InsertDriveFileIcon from "@mui/icons-material/InsertDriveFile";

import { releaseDate } from "./releaseDate";

import ardaOnalPhoto from "./static/images/avatar/ardaOnal.jpg";
import efeBeydoganPhoto from "./static/images/avatar/efeBeydogan.jpg";
import mertBarkinErPhoto from "./static/images/avatar/mertBarkinEr.jpg";
import erenPolatPhoto from "./static/images/avatar/erenPolat.jpg";
import emirMelihErdemPhoto from "./static/images/avatar/emirMelihErdem.jpg";
import ozgurOguzPhoto from "./static/images/avatar/ozgurOguz.jpg";
import testReport from "./static/files/reports/test_report.pdf";

const paddingMarginStyle: React.CSSProperties = { margin: 16, padding: 16 };
const appBarSectionStyle = {
  mr: 4,
  color: "inherit",
  textDecoration: "none",
};
const avatarStyle = { width: 56, height: 56, mr: 4 };

const mdTheme = createTheme({
  palette: {
    mode: "dark",
  },
  typography: {
    fontFamily: "monospace",
  },
});

const pageMaxWidth = 1200;

function App() {
  return (
    <ThemeProvider theme={mdTheme}>
      <CssBaseline />
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
        }}
      >
        <AppBar position="static">
          <Container>
            <Toolbar disableGutters>
              <Typography
                variant="h4"
                noWrap
                sx={{
                  mr: 8,
                  display: { xs: "none", md: "flex" },
                  fontFamily: "monospace",
                  fontWeight: 700,
                  letterSpacing: ".3rem",
                  color: "inherit",
                  textDecoration: "none",
                }}
              >
                R-DA🤖
              </Typography>

              <Typography
                variant="h6"
                noWrap
                component="a"
                href="#about"
                sx={appBarSectionStyle}
              >
                About
              </Typography>

              <Typography
                variant="h6"
                noWrap
                component="a"
                href="#status"
                sx={appBarSectionStyle}
              >
                Status
              </Typography>

              <Typography
                variant="h6"
                noWrap
                component="a"
                href="#team"
                sx={appBarSectionStyle}
              >
                Team
              </Typography>

              <Typography
                variant="h6"
                noWrap
                component="a"
                href="#project-reports"
                sx={appBarSectionStyle}
              >
                Reports
              </Typography>

              <Typography
                variant="h6"
                noWrap
                component="a"
                href="#supervisor"
                sx={appBarSectionStyle}
              >
                Supervisor
              </Typography>
            </Toolbar>
          </Container>
        </AppBar>
        <Box
          component="main"
          display="flex"
          style={{ flexDirection: "column" }}
          alignItems="center"
          alignSelf="center"
          maxWidth={pageMaxWidth}
        >
          <Box style={paddingMarginStyle}>
            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="about"></div>
              <Typography variant="h2"> About</Typography>
              <Typography
                width="100%"
                textAlign="center"
                style={{
                  display: "flex",
                  alignItems: "center",
                  flexDirection: "column",
                }}
              >
                R-DA (Robot for Delivery of Aliments) was founded on October 20,
                1984 by İhsan Doğramacı (1915 – 2010) through the joint
                resolution of the İhsan Doğramacı Education Foundation, the
                İhsan Doğramacı Science and Research Foundation, and the İhsan
                Doğramacı Health Foundation. The aim was to create a center of
                excellence in higher education and research. The name “Bilkent”
                exemplifies the founder’s aim, since it is an acronym of bilim
                kenti Turkish for “city of science and knowledge.” The
                university is located in Turkey’s capital city of Ankara. The
                founder, himself an academic, had earlier contributed to the
                establishment of numerous public institutions of higher learning
                and served as rector of Ankara University, as chairman of the
                Board of Trustees of Middle East Technical University and as
                founder and first rector of Hacettepe University. It had long
                been his objective to establish a private, non-profit university
                distinguished by its high quality education and research. During
                the time he spent at Harvard and Washington universities in the
                United States he had observed the advantages of independently
                endowed non-profit research universities that serve the public
                through higher education. With these in mind he advocated for
                decades for the Turkish legal system to allow such institutions,
                and when this dream finally materialized, he established Bilkent
                University along the same lines.
              </Typography>
            </Paper>

            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="status">
                <Typography variant="h2"> Status</Typography>
              </div>
              <List>
                <ListItem>
                  <Checkbox disabled={true} />
                  <ListItemText primary="Project Specification Document"></ListItemText>
                </ListItem>

                <ListItem>
                  <Checkbox disabled={true} />
                  <ListItemText primary="Requirement Gathering"></ListItemText>
                </ListItem>

                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Propose project to coordinators"></ListItemText>
                </ListItem>
              </List>
            </Paper>

            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="team">
                <Typography variant="h2"> Team</Typography>
              </div>
              <List>
                <ListItem>
                  <a href="https://www.linkedin.com/in/ardaonal/">
                    <ListItemAvatar>
                      <Avatar
                        alt="Arda Onal"
                        src={ardaOnalPhoto}
                        sx={avatarStyle}
                      />
                    </ListItemAvatar>
                  </a>
                  <ListItemText primary="Arda Önal" />
                </ListItem>

                <ListItem>
                  <a href="https://www.linkedin.com/in/efebeydogan/">
                    <ListItemAvatar>
                      <Avatar
                        alt="Efe Beydogan"
                        src={efeBeydoganPhoto}
                        sx={avatarStyle}
                      />
                    </ListItemAvatar>
                  </a>
                  <ListItemText primary="Efe Beydogan" />
                </ListItem>

                <ListItem>
                  <a href="https://www.linkedin.com/in/emir-melih-erdem/">
                    <ListItemAvatar>
                      <Avatar
                        alt="Emir Melih Erdem"
                        src={emirMelihErdemPhoto}
                        sx={avatarStyle}
                      />
                    </ListItemAvatar>
                  </a>
                  <ListItemText primary="Emir Melih Erdem" />
                </ListItem>

                <ListItem>
                  <a href="https://www.linkedin.com/in/eren-polat323/">
                    <ListItemAvatar>
                      <Avatar
                        alt="Eren Polat"
                        src={erenPolatPhoto}
                        sx={avatarStyle}
                      />
                    </ListItemAvatar>
                  </a>
                  <ListItemText primary="Eren Polat" />
                </ListItem>

                <ListItem>
                  <a href="https://www.linkedin.com/in/mertbarkın/">
                    <ListItemAvatar>
                      <Avatar
                        alt="Mert Barkin Er"
                        src={mertBarkinErPhoto}
                        sx={avatarStyle}
                      />
                    </ListItemAvatar>
                  </a>
                  <ListItemText
                    primary="Mert Barkın Er"
                    onClick={() =>
                      window.open("https://youtu.be/wkygaWV_Gyw?t=47")
                    }
                  />
                </ListItem>
              </List>
            </Paper>

            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="project-reports">
                <Typography variant="h2"> Project reports</Typography>
              </div>
              <List>
                <ListItem>
                  <ListItemIcon>
                    <InsertDriveFileIcon />
                  </ListItemIcon>
                  <a href={testReport} download="Test_Filename">
                    <Typography
                      sx={{
                        color: "white",
                        fontSize: "24px",
                        textDecoration: "underline",
                        textDecorationColor: "white",
                      }}
                    >
                      Test Project Specification Document
                    </Typography>
                  </a>
                </ListItem>
              </List>
            </Paper>

            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="supervisor">
                <Typography variant="h4"> Supervisor</Typography>
              </div>
              <ListItem>
                <a href="https://oz-oguz.github.io">
                  <ListItemAvatar>
                    <Avatar
                      alt="Ozgur Oguz"
                      src={ozgurOguzPhoto}
                      sx={avatarStyle}
                    />
                  </ListItemAvatar>
                </a>
                <ListItemText primary="Ozgur S. Oguz" />
              </ListItem>

              <Typography variant="h5"> Jury Members</Typography>
              <Typography> TBD</Typography>

              <Typography variant="h5"> Innovation Expert</Typography>
              <Typography> TBD</Typography>
            </Paper>
          </Box>
        </Box>

        <AppBar sx={{ top: "auto", bottom: 0 }}>
          <Box style={{ display: "flex", justifyContent: "space-between" }}>
            <Typography>Last updated: {releaseDate}</Typography>
            <Typography>
              &#169; {new Date().getFullYear()} Beetirme. All rights reserved.
            </Typography>
          </Box>
        </AppBar>
      </Box>
    </ThemeProvider>
  );
}
export default App;

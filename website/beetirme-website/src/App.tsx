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
import erhanDolakPhoto from "./static/images/avatar/erhanDolak.jpg";
import tagmacTopalPhoto from "./static/images/avatar/tagmacTopal.jpg";
import iiwaPhoto from "./static/images/robot/iiwa.jpg";
import groupPhoto from "./static/images/group_photo.jpg";

import specReport from "./static/files/reports/T2323_Project_Specification_Document.pdf";
import analReport from "./static/files/reports/T2323_Analysis_Requirements_Report.pdf";
import designReport from "./static/files/reports/T2323_Detail_Design_Document.pdf";
import finalReport from "./static/files/reports/T2323_FinalReport.pdf";

import demo1 from "./static/files/presentations/DemoPresentation.pdf";
import demo2 from "./static/files/presentations/FinalDemoPresentation.pdf";

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
                  mr: 4,
                  display: { xs: "none", md: "flex" },
                  fontFamily: "monospace",
                  fontWeight: 700,
                  letterSpacing: ".3rem",
                  color: "inherit",
                  textDecoration: "none",
                }}
              >
                GroceryBee🐝
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
              <Typography variant="h3"> About</Typography>
              <Typography
                width="100%"
                textAlign="center"
                style={{
                  display: "flex",
                  alignItems: "center",
                  flexDirection: "column",
                }}
              >
                GroceryBee is a robot to assist grocery stores in keeping up
                with their online orders. GroceryBee will help grocery store
                employees by collecting the items which are required for an
                online order from the store shelves, thus speeding up the
                delivery process. GroceryBee is designed to make decisions in an
                uncertain environment, so it is able to exploit real-time
                information from RGB-D cameras to make real-time changes in
                planning. GroceryBee is able to collect required items in
                response to the orders while navigating between the supermarket
                aisles without crashing. It can easily detect requested items on
                the shelves, picks them up safely and brings them to the
                delivery point for speedy delivery to customers.
              </Typography>
              <br></br>
              <Typography variant="h5">Repository</Typography>
              <a href="https://github.com/ardaOnal/grocery-bee" target="_blank">
                <Typography
                  sx={{
                    color: "white",
                    textDecoration: "underline",
                    textDecorationColor: "white",
                  }}
                >
                  https://github.com/ardaOnal/grocery-bee
                </Typography>
              </a>
            </Paper>

            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="status">
                <Typography variant="h3"> Status</Typography>
              </div>
              <br></br>
              <div
                style={{
                  display: "flex",
                  alignItems: "center",
                  flexDirection: "column",
                }}
              >
                <iframe
                  width={"70%"}
                  height={360}
                  src="https://www.youtube.com/embed/afYTiJdF9Yk?autoplay=1"
                ></iframe>
                <Typography>GroceryBee demo video</Typography>
              </div>

              <br></br>
              <List>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Merge everything and create final product"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Implementation of Environment Generation"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Implementation of Mobile Navigation"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Implementation of Perception"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Implementation of Pick and Place with Cameras"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Learning Technologies"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Analysis and Requirements Report"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Requirement Analysis"></ListItemText>
                </ListItem>
                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
                  <ListItemText primary="Project Specification Report"></ListItemText>
                </ListItem>

                <ListItem>
                  <Checkbox defaultChecked disabled={true} />
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
                <Typography variant="h3"> Team</Typography>
              </div>

              <div
                style={{
                  alignItems: "center",
                  display: "flex",
                  flexDirection: "column",
                }}
              >
                <img src={groupPhoto} width={"70%"}></img>
                <Typography>
                  From left to right: Arda Önal, Emir Melih Erdem, Mert Barkın
                  Er, Efe Beydoğan (robo), Eren Polat
                </Typography>
              </div>
              <List>
                <ListItem>
                  <a
                    href="https://www.linkedin.com/in/ardaonal/"
                    target="_blank"
                  >
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
                  <a
                    href="https://www.linkedin.com/in/efebeydogan/"
                    target="_blank"
                  >
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
                  <a
                    href="https://www.linkedin.com/in/emir-melih-erdem/"
                    target="_blank"
                  >
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
                  <a
                    href="https://www.linkedin.com/in/eren-polat323/"
                    target="_blank"
                  >
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
                  <a
                    href="https://www.linkedin.com/in/mertbarkın/"
                    target="_blank"
                  >
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
                <Typography variant="h3"> Project reports</Typography>
              </div>
              <List>
                <ListItem>
                  <ListItemIcon>
                    <InsertDriveFileIcon />
                  </ListItemIcon>
                  <a href={analReport} target="_blank">
                    <Typography
                      sx={{
                        color: "white",
                        fontSize: "24px",
                        textDecoration: "underline",
                        textDecorationColor: "white",
                      }}
                    >
                      Analysis and Requirements Document
                    </Typography>
                  </a>
                </ListItem>
                <ListItem>
                  <ListItemIcon>
                    <InsertDriveFileIcon />
                  </ListItemIcon>
                  <a href={specReport} target="_blank">
                    <Typography
                      sx={{
                        color: "white",
                        fontSize: "24px",
                        textDecoration: "underline",
                        textDecorationColor: "white",
                      }}
                    >
                      Project Specification Document
                    </Typography>
                  </a>
                </ListItem>
                <ListItem>
                  <ListItemIcon>
                    <InsertDriveFileIcon />
                  </ListItemIcon>
                  <a href={designReport} target="_blank">
                    <Typography
                      sx={{
                        color: "white",
                        fontSize: "24px",
                        textDecoration: "underline",
                        textDecorationColor: "white",
                      }}
                    >
                      Detailed Design Document
                    </Typography>
                  </a>
                </ListItem>
                <ListItem>
                  <ListItemIcon>
                    <InsertDriveFileIcon />
                  </ListItemIcon>
                  <a href={finalReport} target="_blank">
                    <Typography
                      sx={{
                        color: "white",
                        fontSize: "24px",
                        textDecoration: "underline",
                        textDecorationColor: "white",
                      }}
                    >
                      Final Report
                    </Typography>
                  </a>
                </ListItem>
              </List>
            </Paper>

            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="demo-presentation-slides">
                <Typography variant="h3"> Demo presentation slides</Typography>
              </div>
              <List>
                <ListItem>
                  <ListItemIcon>
                    <InsertDriveFileIcon />
                  </ListItemIcon>
                  <a href={demo1} target="_blank">
                    <Typography
                      sx={{
                        color: "white",
                        fontSize: "24px",
                        textDecoration: "underline",
                        textDecorationColor: "white",
                      }}
                    >
                      CS491 Final Demo Slides
                    </Typography>
                  </a>
                </ListItem>
                <ListItem>
                  <ListItemIcon>
                    <InsertDriveFileIcon />
                  </ListItemIcon>
                  <a href={demo2} target="_blank">
                    <Typography
                      sx={{
                        color: "white",
                        fontSize: "24px",
                        textDecoration: "underline",
                        textDecorationColor: "white",
                      }}
                    >
                      CS492 Final Demo Slides
                    </Typography>
                  </a>
                </ListItem>
              </List>
            </Paper>

            <Paper elevation={24} style={paddingMarginStyle}>
              <div id="supervisor">
                <Typography variant="h5"> Supervisor</Typography>
              </div>
              <ListItem>
                <a href="https://oz-oguz.github.io" target="_blank">
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

              <Typography variant="h5"> Innovation Expert</Typography>
              <ListItem>
                <a href="https://oz-oguz.github.io" target="_blank">
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
              <ListItem>
                <a
                  href="https://www.linkedin.com/in/erhandolak/"
                  target="_blank"
                >
                  <ListItemAvatar>
                    <Avatar
                      alt="Erhan Dolak"
                      src={erhanDolakPhoto}
                      sx={avatarStyle}
                    />
                  </ListItemAvatar>
                </a>
                <ListItemText primary="Erhan Dolak" />
              </ListItem>
              <ListItem>
                <a
                  href="https://www.linkedin.com/in/tagmac-topal-06a8879/"
                  target="_blank"
                >
                  <ListItemAvatar>
                    <Avatar
                      alt="Tagmac Topal"
                      src={tagmacTopalPhoto}
                      sx={avatarStyle}
                    />
                  </ListItemAvatar>
                </a>
                <ListItemText primary="Tagmac Topal" />
              </ListItem>
              <ListItem>
                <a href="https://oz-oguz.github.io" target="_blank">
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
            </Paper>
          </Box>
        </Box>

        <AppBar sx={{ top: "auto", bottom: 0 }}>
          <Box style={{ display: "flex", justifyContent: "space-between" }}>
            <Typography>Last updated: {releaseDate}</Typography>
            <Typography>
              &#169; {new Date().getFullYear()} GroceryBee. All rights reserved.
            </Typography>
          </Box>
        </AppBar>
      </Box>
    </ThemeProvider>
  );
}
export default App;

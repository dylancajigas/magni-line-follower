using System;
using System.Collections.Generic;
using System.IO;
using System.Windows.Forms;

namespace BarcodeGenerator
{
    public partial class Graphical : Form
    {
        private static readonly string[] SETTINGS_GROUP_STATIONS = { "group1Stations=", "group2Stations=", "group3Stations=", "group4Stations=", "group5Stations=" };
        private static readonly string[] SETTINGS_GROUP_NAMES = { "group1Name=", "group2Name=", "group3Name=", "group4Name=", "group5Name=" };
        private static readonly int NUM_GROUPS = 5;
        private static readonly int DEFAULT_GROUP_STATIONS = 0;

        // internal variable setup
        private int[] groupsNumStations;
        private List<bool[]> stopStations;
        private string[] groupNames;
        private List<Button> buttons;
        private List<string> returns; // 0th is length, 1st is stops

        public Graphical(List<string> returns)
        {
            InitializeComponent();

            // set up internal variables
            this.returns = returns;
            groupsNumStations = new int[NUM_GROUPS + 1];
            groupNames = new string[NUM_GROUPS];
            stopStations = new List<bool[]>
            {
                new bool[10],
                new bool[10],
                new bool[10],
                new bool[10],
                new bool[10]
            };
            buttons = new List<Button>
            {
                group1Button,
                group2Button,
                group3Button,
                group4Button,
                group5Button
            };

            // Attempt to read settings from cfg file
            if (File.Exists("settings.cfg"))
            {
                // Read settings and set variables
                groupsNumStations = new int[NUM_GROUPS + 1];
                StreamReader reader = new StreamReader("settings.cfg");
                string line = reader.ReadLine();

                // attempt to parse lines
                if(line == null || !line.Contains("consecutive="))
                {
                    MessageBox.Show("Settings file has bad format!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    LoadDefaultSettings();
                }
                else
                {
                    groupsNumStations[NUM_GROUPS] = line.Substring("consecutive=".Length) == "True" ? 1 : 0;
                    for (int i = 0; i < NUM_GROUPS * 2; i++)
                    {
                        if (i < NUM_GROUPS)
                        {
                            line = reader.ReadLine();
                            if (line == null || !line.Contains(SETTINGS_GROUP_NAMES[i]))
                            {
                                MessageBox.Show("Settings file has bad format!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                                LoadDefaultSettings();
                                break;
                            }
                            groupNames[i] = line.Substring(SETTINGS_GROUP_NAMES[i].Length);
                        }
                        else
                        {
                            line = reader.ReadLine();
                            if (line == null ||
                                !int.TryParse(line.Substring(SETTINGS_GROUP_STATIONS[i - NUM_GROUPS].Length), out int curr) ||
                                !line.Contains(SETTINGS_GROUP_STATIONS[i - NUM_GROUPS]))
                            {
                                MessageBox.Show("Settings file has bad format!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                                LoadDefaultSettings();
                                break;
                            }
                            groupsNumStations[i - NUM_GROUPS] = curr;
                        }
                    }
                }
                
                reader.Close();
            }
            else
            {
                LoadDefaultSettings();
            }

            //edit button text
            for (int i = 0; i < NUM_GROUPS; i++)
            {
                buttons[i].Text = groupNames[i];
            }

            for (int i = 0; i < NUM_GROUPS; i++)
            {
                buttons[i].Enabled = groupsNumStations[i] > 0;
            }
        }

        private void settingsButton_Click(object sender, EventArgs e)
        {
            // open settings window
            GraphicalSettings settings = new GraphicalSettings(groupsNumStations, groupNames);
            settings.ShowDialog();
            for(int i = 0; i < NUM_GROUPS; i++)
            {
                buttons[i].Text = groupNames[i];
            }
            for(int i = 0; i < NUM_GROUPS; i++)
            {
                buttons[i].Enabled = groupsNumStations[i] > 0;
            }
        }


        // open submenus based based on button clicked
        private void group1Button_Click(object sender, EventArgs e)
        {
            GraphicalSubMenu sm = new GraphicalSubMenu(groupNames[0], groupsNumStations[0], stopStations[0], groupsNumStations[NUM_GROUPS] == 1, 0);
            sm.ShowDialog();
        }

        private void group2Button_Click(object sender, EventArgs e)
        {
            int prevStations = groupsNumStations[0];
            GraphicalSubMenu sm = new GraphicalSubMenu(groupNames[1], groupsNumStations[1], stopStations[1], groupsNumStations[NUM_GROUPS] == 1, prevStations);
            sm.ShowDialog();
        }

        private void group3Button_Click(object sender, EventArgs e)
        {
            int prevStations = groupsNumStations[0] + groupsNumStations[1];
            GraphicalSubMenu sm = new GraphicalSubMenu(groupNames[2], groupsNumStations[2], stopStations[2], groupsNumStations[NUM_GROUPS] == 1, prevStations);
            sm.ShowDialog();
        }

        private void group4Button_Click(object sender, EventArgs e)
        {
            int prevStations = groupsNumStations[0] + groupsNumStations[1] + groupsNumStations[2];
            GraphicalSubMenu sm = new GraphicalSubMenu(groupNames[3], groupsNumStations[3], stopStations[3], groupsNumStations[NUM_GROUPS] == 1, prevStations);
            sm.ShowDialog();
        }

        private void group5Button_Click(object sender, EventArgs e)
        {
            int prevStations = groupsNumStations[0] + groupsNumStations[1] + groupsNumStations[2] + groupsNumStations[3];
            GraphicalSubMenu sm = new GraphicalSubMenu(groupNames[4], groupsNumStations[4], stopStations[4], groupsNumStations[NUM_GROUPS] == 1, prevStations);
            sm.ShowDialog();
        }

        private void generateButton_Click(object sender, EventArgs e)
        {
            // save variables to "return" array references
            var totalLength = 0;
            var stopsStr = "";
            for (int g = 0; g < NUM_GROUPS; g++)
            {
                for (int i = 0; i < groupsNumStations[g]; i++)
                {
                    if (stopStations[g][i])
                    {
                        stopsStr += (totalLength + i + 1).ToString() + ",";
                    }
                }
                totalLength += groupsNumStations[g];
            }
            if(stopsStr.Length <= 0)
            {
                MessageBox.Show("Select at least one stop!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            stopsStr = stopsStr.Substring(0, stopsStr.Length - 1);

            returns[0] = totalLength.ToString();
            returns[1] = stopsStr;
            this.Close();
        }

        private void cancelButton_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void LoadDefaultSettings()
        {
            // set up variables in default if settings file couldn't be loaded
            for (int i = 0; i < NUM_GROUPS; i++)
            {
                groupsNumStations[i] = DEFAULT_GROUP_STATIONS;
                groupNames[i] = "Group " + (i + 1);
            }
            groupsNumStations[NUM_GROUPS] = 0;
        }
    }
}

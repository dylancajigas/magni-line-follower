using System;
using System.Collections.Generic;
using System.IO;
using System.Windows.Forms;

namespace BarcodeGenerator
{
    public partial class GraphicalSettings : Form
    {
        private static readonly string[] SETTINGS_GROUP_STATIONS = { "group1Stations=", "group2Stations=", "group3Stations=", "group4Stations=", "group5Stations=" };
        private static readonly string[] SETTINGS_GROUP_NAMES = { "group1Name=", "group2Name=", "group3Name=", "group4Name=", "group5Name=" };
        private static readonly int NUM_GROUPS = 5;
        private static readonly int DEFAULT_GROUP_STATIONS = 0;

        // internal variables
        private int[] groupsNumStations;
        private string[] groupNames;
        private int[] groupsNumStationsRet;
        private string[] groupNamesRet;
        private List<TextBox> groupNameTextBoxes;
        private List<TextBox> groupStationsTextBoxes;
        public GraphicalSettings(int[] groupsNumStationsRet, string[] groupNamesRet)
        {
            InitializeComponent();

            // set up interal variables
            groupsNumStations = new int[NUM_GROUPS];
            groupNames = new string[NUM_GROUPS];
            this.groupsNumStationsRet = groupsNumStationsRet;
            this.groupNamesRet = groupNamesRet;
            groupNameTextBoxes = new List<TextBox>
            {
                group1NameTextBox,
                group2NameTextBox,
                group3NameTextBox,
                group4NameTextBox,
                group5NameTextBox
            };
            groupStationsTextBoxes = new List<TextBox>
            {
                group1StationsTextBox,
                group2StationsTextBox,
                group3StationsTextBox,
                group4StationsTextBox,
                group5StationsTextBox
            };

            // Attempt to read settings from cfg file
            if (File.Exists("settings.cfg"))
            {
                // Read settings and set variables
                groupsNumStations = new int[NUM_GROUPS + 1];
                StreamReader reader = new StreamReader("settings.cfg");
                string line = reader.ReadLine();
                if (line == null || !line.Contains("consecutive="))
                {
                    MessageBox.Show("Settings file has bad format!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    LoadDefaultSettings();
                }
                else
                {
                    consecutiveCheckBox.Checked = line.Substring("consecutive=".Length) == "True";
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
                                !line.Contains(SETTINGS_GROUP_STATIONS[i - NUM_GROUPS]) ||
                                !int.TryParse(line.Substring(SETTINGS_GROUP_STATIONS[i - NUM_GROUPS].Length), out int curr))
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

            // Initialize the text boxes
            for(int i = 0; i < NUM_GROUPS; i++)
            {
                groupNameTextBoxes[i].Text = groupNames[i];
                groupStationsTextBoxes[i].Text = groupsNumStations[i].ToString();
            }
        }

        private void saveButton_Click(object sender, EventArgs e)
        {
            /*
             * FORMAT:
             * consecutive=
             * group1Name=
             * group2Name=
             * group3Name=
             * group4Name=
             * group5Name=
             * group1Stations=
             * group2Stations= 
             * group3Stations= 
             * group4Stations= 
             * group5Stations= 
             */

            // Save textbox entries to variables
            for(int i = 0; i < NUM_GROUPS; i++)
            {
                if (!int.TryParse(groupStationsTextBoxes[i].Text, out int curr) || groupNameTextBoxes[i].Text.Length <= 0 || curr < 0)
                {
                    MessageBox.Show("Invalid entries for group " + (i+1) + "!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
                groupsNumStations[i] = curr;
                groupNames[i] = groupNameTextBoxes[i].Text;
            }

            // Write settings to file
            StreamWriter writer = new StreamWriter("settings.cfg");
            writer.WriteLine("consecutive=" + consecutiveCheckBox.Checked);
            for (int i = 0; i < NUM_GROUPS; i++)
            {
                writer.WriteLine(SETTINGS_GROUP_NAMES[i] + groupNames[i]);
            }
            for(int i = 0; i < NUM_GROUPS; i++)
            {
                writer.WriteLine(SETTINGS_GROUP_STATIONS[i] + groupsNumStations[i]);
            }
            writer.Close();

            // save variables to "return" array references
            for(int i = 0; i < NUM_GROUPS; i++)
            {
                groupNamesRet[i] = groupNames[i];
                groupsNumStationsRet[i] = groupsNumStations[i];
            }
            groupsNumStationsRet[NUM_GROUPS] = consecutiveCheckBox.Checked ? 1 : 0;
            this.Close();
        }

        private void cancelButton_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void LoadDefaultSettings()
        {
            // load default settings if settings couldn't be read
            for (int i = 0; i < NUM_GROUPS; i++)
            {
                groupsNumStations[i] = DEFAULT_GROUP_STATIONS;
                groupNames[i] = "Group " + (i + 1);
            }
            groupsNumStations[NUM_GROUPS] = 0;
        }
    }
}

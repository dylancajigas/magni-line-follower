using System;
using System.Collections.Generic;
using System.Windows.Forms;

namespace BarcodeGenerator
{
    public partial class GraphicalSubMenu : Form
    {
        private static int NUM_STATIONS = 10;

        // internal variable setup
        private bool[] checkedReference;
        private int numStations;
        private List<CheckBox> checkboxes;
        bool consecutive;
        int prevStations;



        public GraphicalSubMenu(string title, int numStations, bool[] boxesChecked, bool consecutive, int prevStations)
        {
            InitializeComponent();

            // change title of the window
            this.Text = title;

            // set up variables
            checkboxes = new List<CheckBox>
            {
                st1CheckBox,
                st2CheckBox,
                st3CheckBox,
                st4CheckBox,
                st5CheckBox,
                st6CheckBox,
                st7CheckBox,
                st8CheckBox,
                st9CheckBox,
                st10CheckBox
            };

            this.checkedReference = boxesChecked;
            this.numStations = numStations;

            // enable correct number of boxes, change name based on settings
            for (int i = 0; i < numStations; i++)
            {
                if (consecutive)
                {
                    checkboxes[i].Text = "Station " + (prevStations + i + 1); 
                }
                checkboxes[i].Checked = boxesChecked[i];
                checkboxes[i].Enabled = true;
                checkboxes[i].Visible = true;
            }
            for(int i = numStations; i < 10; i++)
            {
                checkboxes[i].Enabled = false;
                checkboxes[i].Visible = false;
            }

            // change size of window based on number of stations
            this.Height = (numStations * 21) + 80;
            cancelButton.Location = new System.Drawing.Point(9, this.Height - 70);
            saveButton.Location = new System.Drawing.Point(15 + cancelButton.Width, this.Height - 70);
        }

        private void saveButton_Click(object sender, EventArgs e)
        {
            // save checked boxes to return array
            for(int i = 0; i < NUM_STATIONS; i++)
            {
                checkedReference[i] = checkboxes[i].Checked;
            }
            this.Close();
        }

        private void cancelButton_Click(object sender, EventArgs e)
        {
            this.Close();
        }
    }
}

namespace BarcodeGenerator
{
    partial class Graphical
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.settingsButton = new System.Windows.Forms.Button();
            this.group1Button = new System.Windows.Forms.Button();
            this.homeLabel = new System.Windows.Forms.Label();
            this.group2Button = new System.Windows.Forms.Button();
            this.group3Button = new System.Windows.Forms.Button();
            this.group4Button = new System.Windows.Forms.Button();
            this.backgroundLabel = new System.Windows.Forms.Label();
            this.generateButton = new System.Windows.Forms.Button();
            this.cancelButton = new System.Windows.Forms.Button();
            this.group5Button = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // settingsButton
            // 
            this.settingsButton.Location = new System.Drawing.Point(12, 319);
            this.settingsButton.Name = "settingsButton";
            this.settingsButton.Size = new System.Drawing.Size(75, 30);
            this.settingsButton.TabIndex = 6;
            this.settingsButton.Text = "Settings";
            this.settingsButton.UseVisualStyleBackColor = true;
            this.settingsButton.Click += new System.EventHandler(this.settingsButton_Click);
            // 
            // group1Button
            // 
            this.group1Button.Location = new System.Drawing.Point(345, 137);
            this.group1Button.Name = "group1Button";
            this.group1Button.Size = new System.Drawing.Size(114, 46);
            this.group1Button.TabIndex = 1;
            this.group1Button.Text = "group1";
            this.group1Button.UseVisualStyleBackColor = true;
            this.group1Button.Click += new System.EventHandler(this.group1Button_Click);
            // 
            // homeLabel
            // 
            this.homeLabel.AutoSize = true;
            this.homeLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.homeLabel.Location = new System.Drawing.Point(372, 267);
            this.homeLabel.Name = "homeLabel";
            this.homeLabel.Size = new System.Drawing.Size(64, 25);
            this.homeLabel.TabIndex = 2;
            this.homeLabel.Text = "Home";
            // 
            // group2Button
            // 
            this.group2Button.Location = new System.Drawing.Point(345, 24);
            this.group2Button.Name = "group2Button";
            this.group2Button.Size = new System.Drawing.Size(114, 46);
            this.group2Button.TabIndex = 2;
            this.group2Button.Text = "group2";
            this.group2Button.UseVisualStyleBackColor = true;
            this.group2Button.Click += new System.EventHandler(this.group2Button_Click);
            // 
            // group3Button
            // 
            this.group3Button.Location = new System.Drawing.Point(28, 24);
            this.group3Button.Name = "group3Button";
            this.group3Button.Size = new System.Drawing.Size(114, 46);
            this.group3Button.TabIndex = 3;
            this.group3Button.Text = "group3";
            this.group3Button.UseVisualStyleBackColor = true;
            this.group3Button.Click += new System.EventHandler(this.group3Button_Click);
            // 
            // group4Button
            // 
            this.group4Button.Location = new System.Drawing.Point(28, 137);
            this.group4Button.Name = "group4Button";
            this.group4Button.Size = new System.Drawing.Size(114, 46);
            this.group4Button.TabIndex = 4;
            this.group4Button.Text = "group4";
            this.group4Button.UseVisualStyleBackColor = true;
            this.group4Button.Click += new System.EventHandler(this.group4Button_Click);
            // 
            // backgroundLabel
            // 
            this.backgroundLabel.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.backgroundLabel.Location = new System.Drawing.Point(81, 47);
            this.backgroundLabel.Name = "backgroundLabel";
            this.backgroundLabel.Size = new System.Drawing.Size(323, 233);
            this.backgroundLabel.TabIndex = 3;
            // 
            // generateButton
            // 
            this.generateButton.Location = new System.Drawing.Point(378, 319);
            this.generateButton.Name = "generateButton";
            this.generateButton.Size = new System.Drawing.Size(81, 30);
            this.generateButton.TabIndex = 8;
            this.generateButton.Text = "Done";
            this.generateButton.UseVisualStyleBackColor = true;
            this.generateButton.Click += new System.EventHandler(this.generateButton_Click);
            // 
            // cancelButton
            // 
            this.cancelButton.Location = new System.Drawing.Point(291, 319);
            this.cancelButton.Name = "cancelButton";
            this.cancelButton.Size = new System.Drawing.Size(81, 30);
            this.cancelButton.TabIndex = 7;
            this.cancelButton.Text = "Cancel";
            this.cancelButton.UseVisualStyleBackColor = true;
            this.cancelButton.Click += new System.EventHandler(this.cancelButton_Click);
            // 
            // group5Button
            // 
            this.group5Button.Location = new System.Drawing.Point(28, 259);
            this.group5Button.Name = "group5Button";
            this.group5Button.Size = new System.Drawing.Size(114, 46);
            this.group5Button.TabIndex = 5;
            this.group5Button.Text = "group5";
            this.group5Button.UseVisualStyleBackColor = true;
            this.group5Button.Click += new System.EventHandler(this.group5Button_Click);
            // 
            // Graphical
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(485, 394);
            this.ControlBox = false;
            this.Controls.Add(this.homeLabel);
            this.Controls.Add(this.group5Button);
            this.Controls.Add(this.group4Button);
            this.Controls.Add(this.group3Button);
            this.Controls.Add(this.group2Button);
            this.Controls.Add(this.group1Button);
            this.Controls.Add(this.cancelButton);
            this.Controls.Add(this.generateButton);
            this.Controls.Add(this.settingsButton);
            this.Controls.Add(this.backgroundLabel);
            this.Cursor = System.Windows.Forms.Cursors.Arrow;
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Name = "Graphical";
            this.Text = "Interface";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button settingsButton;
        private System.Windows.Forms.Button group1Button;
        private System.Windows.Forms.Label homeLabel;
        private System.Windows.Forms.Button group2Button;
        private System.Windows.Forms.Button group3Button;
        private System.Windows.Forms.Button group4Button;
        private System.Windows.Forms.Label backgroundLabel;
        private System.Windows.Forms.Button generateButton;
        private System.Windows.Forms.Button cancelButton;
        private System.Windows.Forms.Button group5Button;
    }
}
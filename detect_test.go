package viamfairino

import "testing"

// realFR5SoftwareVersion is the actual GetSoftwareVersion (CMD_ID 905) response
// DATA captured from an FR5 at 10.1.7.1.
const realFR5SoftwareVersion = "FR5-V1-002(V6.0),v3.9.4,V3.9.22-QX"

func TestParseSoftwareVersion(t *testing.T) {
	tests := []struct {
		name                         string
		data                         string
		wantModel, wantWeb, wantCtrl string
	}{
		{"real fr5", realFR5SoftwareVersion, "FR5-V1-002(V6.0)", "v3.9.4", "V3.9.22-QX"},
		{"whitespace", " FR5-X , a , b ", "FR5-X", "a", "b"},
		{"missing fields", "FR5-X", "FR5-X", "", ""},
		{"empty", "", "", "", ""},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			model, web, ctrl := parseSoftwareVersion(tt.data)
			if model != tt.wantModel || web != tt.wantWeb || ctrl != tt.wantCtrl {
				t.Errorf("parseSoftwareVersion(%q) = (%q,%q,%q), want (%q,%q,%q)",
					tt.data, model, web, ctrl, tt.wantModel, tt.wantWeb, tt.wantCtrl)
			}
		})
	}
}

func TestModelFromRobotModel(t *testing.T) {
	tests := []struct {
		robotModel string
		want       string
		wantOK     bool
	}{
		// The real FR5 reports "FR5-V1-002(V6.0)".
		{"FR5-V1-002(V6.0)", ModelFR5, true},
		{"FR5", ModelFR5, true},
		{"fr5-v1", ModelFR5, true},
		// FR5-WML contains "FR5" too, so WML must win.
		{"FR5-WML-V1-001", ModelFR5WML, true},
		{"FR5-wml", ModelFR5WML, true},
		// Unsupported / unknown.
		{"FR3-V1-001", "", false},
		{"FR10", "", false},
		{"", "", false},
	}
	for _, tt := range tests {
		t.Run(tt.robotModel, func(t *testing.T) {
			got, ok := modelFromRobotModel(tt.robotModel)
			if got != tt.want || ok != tt.wantOK {
				t.Errorf("modelFromRobotModel(%q) = (%q,%v), want (%q,%v)",
					tt.robotModel, got, ok, tt.want, tt.wantOK)
			}
		})
	}
}

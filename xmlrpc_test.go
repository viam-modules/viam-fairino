package viamfairino

import (
	"reflect"
	"testing"
)

func TestParseXMLRPCResponseArray(t *testing.T) {
	// Real GetSoftwareVersion response captured from an FR5.
	data := []byte(`<?xml version="1.0" encoding="UTF-8"?>
<methodResponse><params><param><value><array><data>
<value><i4>0</i4></value>
<value><string>FR5-V1-002(V6.0)</string></value>
<value><string>v3.9.4</string></value>
<value><string>V3.9.22-QX</string></value>
</data></array></value></param></params></methodResponse>`)
	vals, err := parseXMLRPCResponse(data)
	if err != nil {
		t.Fatalf("parseXMLRPCResponse: %v", err)
	}
	want := []any{0, "FR5-V1-002(V6.0)", "v3.9.4", "V3.9.22-QX"}
	if !reflect.DeepEqual(vals, want) {
		t.Errorf("got %#v, want %#v", vals, want)
	}
}

func TestParseXMLRPCResponseFault(t *testing.T) {
	// Real fault captured for an undefined method.
	data := []byte(`<?xml version="1.0" encoding="UTF-8"?>
<methodResponse><fault><value><struct>
<member><name>faultCode</name><value><i4>-506</i4></value></member>
<member><name>faultString</name><value><string>Method 'X' not defined</string></value></member>
</struct></value></fault></methodResponse>`)
	if _, err := parseXMLRPCResponse(data); err == nil {
		t.Fatal("expected fault error, got nil")
	}
}

func TestEncodeXMLRPCRequest(t *testing.T) {
	// MoveGripper's argument types: ints + one double (rotNum).
	body, err := encodeXMLRPCRequest("MoveGripper", []any{1, 20, 50, 50, 30000, 1, 0, 0.0, 0, 0})
	if err != nil {
		t.Fatalf("encodeXMLRPCRequest: %v", err)
	}
	s := string(body)
	for _, want := range []string{
		"<methodName>MoveGripper</methodName>",
		"<i4>20</i4>",
		"<double>0</double>",
	} {
		if !contains(s, want) {
			t.Errorf("encoded request missing %q:\n%s", want, s)
		}
	}
}

func contains(haystack, needle string) bool {
	return len(haystack) >= len(needle) && (indexOf(haystack, needle) >= 0)
}

func indexOf(s, sub string) int {
	for i := 0; i+len(sub) <= len(s); i++ {
		if s[i:i+len(sub)] == sub {
			return i
		}
	}
	return -1
}
